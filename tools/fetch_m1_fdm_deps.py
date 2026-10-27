#!/usr/bin/env python3
"""M1-001: fetch JSBSim v1.3.2 and the board's C++ runtime, by pinned hash.

Nothing here is vendored into git. Everything lands in the git-ignored
`third_party/m1/` directory:

  third_party/m1/downloads/         the pinned archives (sha256-verified)
  third_party/m1/jsbsim-1.3.2/      JSBSim source and data (aircraft/, engine/,
                                    systems/, scripts/)
  third_party/m1/gcc14-runtime/     libstdc++ 14 headers + libstdc++.so.6 and
                                    libgcc_s.so.1 to link against
  third_party/m1/fetch_stamp.json   what was extracted, from which hashes

Why the Debian GCC 14 runtime (ISS note in docs/platform/m1-001-build.md):
Zig links its own libc++ statically and with hidden visibility into every
shared object. With libJSBSim.so and libgx_fdm.so each carrying a private
copy, `catch (const std::exception&)` in the shim does not match an
exception thrown inside libJSBSim.so (measured on the board: only
`catch (...)` matched). Building both against the board's own libstdc++.so.6
(Debian 13, 14.2.0-19, byte-identical to the board's installed file) gives
the process one C++ runtime, so typed catches and the error text work.

Re-running is cheap: archives are re-hashed; extraction is skipped when the
stamp matches. Exit 0 ok, 1 on any hash/download/extract failure.
"""
import hashlib, io, json, shutil, sys, tarfile, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / 'third_party' / 'm1'
DL = ROOT / 'downloads'

JSBSIM = dict(
    name='jsbsim-v1.3.2.tar.gz',
    urls=['https://github.com/JSBSim-Team/jsbsim/archive/refs/tags/v1.3.2.tar.gz'],
    sha256='9d3f625ebf59be8b4aa3b6dd4e1db72712dc593685ef4e99f254cfd9603ebc01',
    tag='v1.3.2', tag_commit='14d23722f7f529e62cd488a76e5e8c4320613c3a')

DEB_POOL = 'https://deb.debian.org/debian/pool/main/g/gcc-14/'
DEBS = [
    dict(name='libstdc++-14-dev_14.2.0-19_arm64.deb',
         sha256='8760122d044cd8eb8c69426311d6b5ce7bb8f0f76e9cda1944d153ae878138f8',
         sha1='0ce58dc9b1c0b9fb4bb2d11d2115d3b0efe1a16e'),
    dict(name='libstdc++6_14.2.0-19_arm64.deb',
         sha256='6669b0c52a2e7c6af9adfdabce3ff6e286065cdfbc7b85280862b5f799daebee',
         sha1='4582e5523e677deab65d98eae40f56372f6dc456'),
    dict(name='libgcc-s1_14.2.0-19_arm64.deb',
         sha256='1108bc87879833d6d9a145f22a4a15cddb34e065b4b5f4b97bee586adbac2851',
         sha1='8f3629cf95590aa5492ea87bbfd3ae785c99ebf3'),
]
for d in DEBS:  # deb.debian.org drops superseded versions; snapshot keeps them by hash
    d['urls'] = [DEB_POOL + d['name'], 'https://snapshot.debian.org/file/' + d['sha1']]

# Board files these must equal (sha256 of the board's installed copies,
# Debian 13.1, libstdc++6/libgcc-s1 14.2.0-19, read 2026-10-13).
RUNTIME_LIBS = {
    'libstdc++.so.6': ('./usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.33',
                       'fff1f0050a33ddf29c44634c7639ac16e2573a61e3daab4fcc4c007b432ccf19'),
    'libgcc_s.so.1': ('./usr/lib/aarch64-linux-gnu/libgcc_s.so.1',
                      '9a146967dc38c3820c5dc21c3209812655f66b6551264072b187bd0e787947e0'),
}
HEADER_PREFIXES = {'./usr/include/c++/14/': 'include/c++/14/',
                   './usr/include/aarch64-linux-gnu/c++/14/': 'include/aarch64-linux-gnu/c++/14/'}


def sha256(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def fetch(item):
    dst = DL / item['name']
    if dst.exists() and sha256(dst) == item['sha256']:
        return dst
    DL.mkdir(parents=True, exist_ok=True)
    last = None
    for url in item['urls']:
        try:
            tmp = dst.with_suffix(dst.suffix + '.part')
            with urllib.request.urlopen(url, timeout=120) as r, open(tmp, 'wb') as f:
                shutil.copyfileobj(r, f)
            got = sha256(tmp)
            if got != item['sha256']:
                tmp.unlink()
                last = f'{url}: sha256 {got} != {item["sha256"]}'
                continue
            tmp.replace(dst)
            print(f'fetched {item["name"]} from {url}')
            return dst
        except Exception as e:  # try the next mirror
            last = f'{url}: {e}'
    raise RuntimeError(f'cannot fetch {item["name"]}: {last}')


def deb_data(path):
    b = path.read_bytes()
    if b[:8] != b'!<arch>\n':
        raise RuntimeError(f'{path.name}: not an ar archive')
    o = 8
    while o < len(b):
        h = b[o:o + 60]
        name, size = h[:16].decode().strip().rstrip('/'), int(h[48:58])
        if name.startswith('data.tar'):
            return tarfile.open(fileobj=io.BytesIO(b[o + 60:o + 60 + size]))
        o += 60 + size + (size & 1)
    raise RuntimeError(f'{path.name}: no data.tar member')


def main():
    pins = {'jsbsim': JSBSIM, 'debs': DEBS, 'runtime_libs': RUNTIME_LIBS}
    stamp_path = ROOT / 'fetch_stamp.json'
    files = [fetch(JSBSIM)] + [fetch(d) for d in DEBS]
    want = json.dumps(pins, sort_keys=True)
    if stamp_path.exists():
        try:
            if json.loads(stamp_path.read_text())['pins'] == json.loads(want):
                print(f'up to date: {ROOT}')
                return 0
        except Exception:
            pass
    # Fresh extraction into clean directories.
    src, rt = ROOT / 'jsbsim-1.3.2', ROOT / 'gcc14-runtime'
    for d in (src, rt):
        if d.exists():
            shutil.rmtree(d)
    with tarfile.open(files[0]) as t:
        # GitHub archives carry the tagged commit id as the pax comment.
        if t.pax_headers.get('comment') != JSBSIM['tag_commit']:
            raise RuntimeError(f"archive commit {t.pax_headers.get('comment')} != {JSBSIM['tag_commit']}")
        source_date_epoch = int(t.getmember('jsbsim-1.3.2').mtime)  # the tag commit time
        for m in t.getmembers():
            if Path(m.name).parts[0] != 'jsbsim-1.3.2' or '..' in Path(m.name).parts:
                raise RuntimeError(f'unexpected member {m.name}')
        t.extractall(ROOT, filter='data')
    (rt / 'lib').mkdir(parents=True)
    n_hdr = 0
    for deb in files[1:]:
        with deb_data(deb) as t:
            for m in t.getmembers():
                if m.isreg():
                    for pre, out in HEADER_PREFIXES.items():
                        if m.name.startswith(pre):
                            p = rt / (out + m.name[len(pre):])
                            p.parent.mkdir(parents=True, exist_ok=True)
                            p.write_bytes(t.extractfile(m).read())
                            n_hdr += 1
                for lib, (member, _h) in RUNTIME_LIBS.items():
                    if m.name == member and m.isreg():
                        (rt / 'lib' / lib).write_bytes(t.extractfile(m).read())
    for lib, (_m, h) in RUNTIME_LIBS.items():
        got = sha256(rt / 'lib' / lib)
        if got != h:
            raise RuntimeError(f'{lib}: sha256 {got} != board copy {h}')
    if n_hdr < 500 or not (rt / 'include/aarch64-linux-gnu/c++/14/bits/c++config.h').exists():
        raise RuntimeError(f'libstdc++ headers incomplete ({n_hdr} files)')
    stamp_path.write_text(json.dumps({'pins': json.loads(want), 'header_files': n_hdr,
                                      'jsbsim_source_date_epoch': source_date_epoch}, indent=2) + '\n')
    print(f'extracted JSBSim {JSBSIM["tag"]} and GCC 14 runtime ({n_hdr} headers) into {ROOT}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f'fetch_m1_fdm_deps: {e}', file=sys.stderr)
        raise SystemExit(1)
