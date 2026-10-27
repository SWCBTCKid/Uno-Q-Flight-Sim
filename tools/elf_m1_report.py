#!/usr/bin/env python3
"""M1-001 ELF report: NEEDED, RUNPATH, glibc symbol-version ceiling, exports.

Reuses tools/inspect_elf.py (M0, unchanged) for the header/NEEDED/RUNPATH
parse and adds what M1-001 measure 1/9 needs and inspect_elf.py does not
report: the versions required from each NEEDED library (.gnu.version_r), the
highest GLIBC_x.y among them, and a count of exported (defined, global/weak,
default/protected visibility) dynamic symbols, split into C++ runtime
(std::/__cxa/_Unwind) and other. Dependency-free (no pyelftools/readelf on
the host or the board).

Exit 0 when every file is a valid AArch64 ELF and, if --max-glibc is given,
no file needs a newer GLIBC (--max-glibc) or GLIBCXX (--max-glibcxx)
version; 3 otherwise; 2 on parse failure.
"""
import argparse, json, struct, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inspect_elf import inspect as inspect_m0  # noqa: E402

SHT_SYMTAB, SHT_DYNSYM, SHT_GNU_VERNEED = 2, 11, 0x6ffffffe


def _ver_key(v):
    return tuple(int(x) for x in v.split('.') if x.isdigit())


def report(path):
    b = path.read_bytes()
    base = inspect_m0(path)
    shoff = struct.unpack_from('<Q', b, 40)[0]
    shentsize, shnum, _shstrndx = struct.unpack_from('<HHH', b, 58)
    secs = []
    for i in range(shnum):
        o = shoff + i * shentsize
        name, typ, _fl, _addr, off, size, link, info, _al, entsize = struct.unpack_from('<IIQQQQIIQQ', b, o)
        secs.append(dict(type=typ, off=off, size=size, link=link, info=info, entsize=entsize))

    def cstr(off):
        return b[off:b.index(b'\0', off)].decode('utf-8', 'replace')

    verneed = {}
    for s in secs:
        if s['type'] != SHT_GNU_VERNEED:
            continue
        stro = secs[s['link']]['off']
        o, n = s['off'], s['info']
        for _ in range(n):
            _v, cnt, file_, aux, nxt = struct.unpack_from('<HHIII', b, o)
            lib = cstr(stro + file_)
            a = o + aux
            vs = []
            for _ in range(cnt):
                _h, _f, _other, vname, anext = struct.unpack_from('<IHHII', b, a)
                vs.append(cstr(stro + vname))
                if not anext:
                    break
                a += anext
            verneed[lib] = sorted(set(vs), key=lambda v: (v.rstrip('0123456789.'), _ver_key(v.split('_', 1)[-1])))
            if not nxt:
                break
            o += nxt
    def versions(prefix):
        n = len(prefix)
        return sorted({v for vs in verneed.values() for v in vs if v.startswith(prefix) and v[n:n + 1].isdigit()},
                      key=lambda v: _ver_key(v[n:]))
    glibc, glibcxx = versions('GLIBC_'), versions('GLIBCXX_')
    exports = {'total': 0, 'cxx_runtime': 0, 'gx_fdm': 0, 'other': 0}
    # F-01 forbids JSBSim code from being folded into the harness or shim.
    # Inspect both .dynsym and .symtab: a hidden/local static-link definition
    # is just as much an LGPL boundary breach as an exported definition.
    defined_jsbsim = set()
    for s in secs:
        if s['type'] not in (SHT_SYMTAB, SHT_DYNSYM):
            continue
        stro = secs[s['link']]['off']
        for i in range(1, s['size'] // s['entsize']):
            name, info, other, shndx, _val, _sz = struct.unpack_from('<IBBHQQ', b, s['off'] + i * s['entsize'])
            n = cstr(stro + name)
            if shndx != 0 and 'JSBSim' in n:
                defined_jsbsim.add(n)
            if s['type'] != SHT_DYNSYM:
                continue
            bind, vis = info >> 4, other & 3
            if shndx == 0 or bind not in (1, 2) or vis not in (0, 3):
                continue
            exports['total'] += 1
            if n.startswith('gx_fdm_'):
                exports['gx_fdm'] += 1
            elif n.startswith(('_ZNSt', '_ZNKSt', '_ZSt', '_ZTISt', '_ZTSSt', '_ZTVSt', '__cxa_', '_Unwind_', '__gxx_', '_ZNSt3__1', '_ZTVN10__cxxabiv1')):
                exports['cxx_runtime'] += 1
            else:
                exports['other'] += 1
    base.update(version_needs=verneed, glibc_versions=glibc,
                glibc_max=glibc[-1] if glibc else None,
                glibcxx_max=glibcxx[-1] if glibcxx else None, exports=exports)
    base['defined_jsbsim'] = {
        'count': len(defined_jsbsim),
        'symbols_sample': sorted(defined_jsbsim)[:20],
    }
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path, nargs='+')
    ap.add_argument('--max-glibc', help='fail if any file needs a newer GLIBC_x.y, e.g. 2.34')
    ap.add_argument('--max-glibcxx', help='fail if any file needs a newer GLIBCXX_x.y.z, e.g. 3.4.33 (board libstdc++)')
    ap.add_argument('--forbid-defined-jsbsim', action='store_true',
                    help='fail if an ELF defines a JSBSim C++ symbol (LGPL shared-link boundary)')
    ap.add_argument('--output', type=Path)
    a = ap.parse_args()
    out, bad = [], False
    for p in a.elf:
        try:
            r = report(p)
        except Exception as e:  # parse failure is fatal: never report a partial ceiling
            print(f'ELF report failed for {p}: {e}', file=sys.stderr)
            return 2
        errs = []
        if r['machine'] != 183:
            errs.append('ELF machine is not AArch64')
        if a.max_glibc and r['glibc_max'] and _ver_key(r['glibc_max'][6:]) > _ver_key(a.max_glibc):
            errs.append(f"needs {r['glibc_max']} > GLIBC_{a.max_glibc}")
        if a.max_glibcxx and r['glibcxx_max'] and _ver_key(r['glibcxx_max'][8:]) > _ver_key(a.max_glibcxx):
            errs.append(f"needs {r['glibcxx_max']} > GLIBCXX_{a.max_glibcxx}")
        if a.forbid_defined_jsbsim and r['defined_jsbsim']['count']:
            errs.append(f"defines {r['defined_jsbsim']['count']} JSBSim symbols (must use libJSBSim.so)")
        r['errors'] = errs
        bad |= bool(errs)
        out.append(r)
    text = json.dumps(out, indent=2, sort_keys=True)
    if a.output:
        a.output.write_text(text + '\n', encoding='utf-8')
    print(text)
    return 3 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
