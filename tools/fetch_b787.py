#!/usr/bin/env python3
"""Fetch and verify the FlightGear Aircraft-2024 787-8 alpha input."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import urllib.request
import zipfile

URL = "https://fgaddon.b-cdn.net/Aircraft-2024/787-8.zip"
ARCHIVE_SHA256 = "9061f74b83edeacbbc0b205c128494edb81a29b5b35530e21f1660ddd32018c6"
REQUIRED = {
    "Models/787-8.ac": "c168688d31a0feeab6172541bf78cd50629512c6e9dd57f9ee5dc9128cadffe6",
    "Models/787-8.xml": "dde17a6d62089e6ba2ec7d47b2a8974ddc2dcc787b7b4acf7607468e3dae09f4",
    "Models/Liveries/BOE2.png": "df0cb92fac46addc6abe76e5a6a1a7a19c0251a34b3b9bbff79a52357aec9f5f",
    "Models/BOE.png": "7a7848754e3fb1ade7a8b4f3a93a5a23ed11eda551fd66218a1df2c8e351a517",
    "Models/wings.png": "14ecf52f4b4e2dbd59af18f8dc951150ff8b1fd3522da7b68527fedf03ef669b",
    "Models/landing-gears.png": "d9c8c46e03e579ad0cf2ddbb481180ca2551a0c7d94ca942e40395cbde5e60a3",
    "COPYING": "dcc100d4161cc0b7177545ab6e47216f84857cda3843847c792a25289852dcaa",
    "Authors.txt": "c08bea7b304f9d418534aa6493661936157592659f22201920d48eac1777680e",
}


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(root: pathlib.Path) -> None:
    for relative, expected in REQUIRED.items():
        path = root / relative
        actual = sha256(path) if path.is_file() else "missing"
        if actual != expected:
            raise RuntimeError(f"787 source mismatch: {relative}: {actual} != {expected}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=pathlib.Path, default=pathlib.Path("asset-cache"))
    args = parser.parse_args()
    archive = args.cache / "flightgear-aircraft-2024-787-8.zip"
    root = args.cache / "flightgear-aircraft-2024-787-8" / "787-8"
    args.cache.mkdir(parents=True, exist_ok=True)
    if not archive.is_file() or sha256(archive) != ARCHIVE_SHA256:
        with urllib.request.urlopen(URL, timeout=120) as response, archive.open("wb") as output:
            while block := response.read(1 << 20):
                output.write(block)
    if sha256(archive) != ARCHIVE_SHA256:
        raise RuntimeError("FlightGear 787 archive SHA-256 mismatch")
    if not root.is_dir():
        destination = root.parent
        destination.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(archive) as bundle:
            for member in bundle.infolist():
                target = (destination / member.filename).resolve()
                if destination.resolve() not in target.parents and target != destination.resolve():
                    raise RuntimeError(f"unsafe archive member: {member.filename}")
            bundle.extractall(destination)
    verify(root)
    print(root.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
