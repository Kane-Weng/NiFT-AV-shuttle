#!/usr/bin/env python3
"""
Build the playground's Python bundle, the zip the Pyodide worker unpacks to /nift.

File: build_bundle.py

Description:
  Reads tools/bundle_manifest.txt (one glob per line, relative to the repo root) and writes one
  zip with repo-relative paths, so /nift in the browser looks like the checkout (ARCH.md section 9).
  Next to the zip it writes a small JSON file with the bundle's hash, file count and size. The
  worker uses the hash to tell the browser cache that the bundle changed.

  The zip is reproducible. Files are sorted, and every entry has the same timestamp and mode, so
  the same tree gives the same bytes and the same hash.

  It refuses, and stops with an error on:
    - a file named .env or .env.something anywhere in a path (README section 3, no credentials)
    - test data that must stay out (tools/gen_msgs/golden.json)
    - a path that resolves outside the repo (a symlink or a "..")
    - a manifest line that matches no file (the manifest would be out of date)

  Usage, from anywhere:
    python3 nift_playground/tools/build_bundle.py                 # writes web/public/py/
    python3 nift_playground/tools/build_bundle.py --out DIR
    python3 nift_playground/tools/build_bundle.py --list          # print the files, write nothing
"""

import argparse
import fnmatch
import hashlib
import io
import json
import re
import sys
import zipfile
from pathlib import Path, PurePosixPath

HERE = Path(__file__).resolve().parent
PLAYGROUND = HERE.parent
REPO = PLAYGROUND.parent
MANIFEST = HERE / 'bundle_manifest.txt'
DEFAULT_OUT = PLAYGROUND / 'web' / 'public' / 'py'
ZIP_NAME = 'nift_bundle.zip'
INFO_NAME = 'nift_bundle.json'

# Never bundled, whatever the manifest says.
FORBIDDEN = ('nift_playground/tools/gen_msgs/golden.json',)
ZIP_DATE = (1980, 1, 1, 0, 0, 0)


class BundleError(Exception):
    pass


def expand_braces(pattern):
    """'a/{b,c}.py' -> ['a/b.py', 'a/c.py']. Nested braces work, one level at a time."""
    m = re.search(r'\{([^{}]*)\}', pattern)
    if not m:
        return [pattern]
    out = []
    for alt in m.group(1).split(','):
        out.extend(expand_braces(pattern[:m.start()] + alt + pattern[m.end():]))
    return out


def read_manifest(path=MANIFEST):
    lines = []
    for raw in Path(path).read_text().splitlines():
        line = raw.strip()
        if line and not line.startswith('#'):
            lines.append(line)
    return lines


def is_env_file(rel):
    return any(fnmatch.fnmatch(part, '.env*') for part in PurePosixPath(rel).parts)


def check_path(rel, repo):
    """Raise BundleError if this repo-relative path must not go into the bundle."""
    if is_env_file(rel):
        raise BundleError(f'refusing {rel}: .env files never go into the bundle')
    if rel in FORBIDDEN:
        raise BundleError(f'refusing {rel}: test data, not runtime code')
    if '..' in PurePosixPath(rel).parts or PurePosixPath(rel).is_absolute():
        raise BundleError(f'refusing {rel}: outside the repo')
    real = (repo / rel).resolve()
    try:
        real.relative_to(repo.resolve())
    except ValueError:
        raise BundleError(f'refusing {rel}: resolves outside the repo ({real})') from None


def collect(repo=REPO, manifest=MANIFEST):
    """Return the sorted repo-relative paths the manifest selects. Raises BundleError."""
    repo = Path(repo)
    files = set()
    for line in read_manifest(manifest):
        for pattern in expand_braces(line):
            if '..' in PurePosixPath(pattern).parts or PurePosixPath(pattern).is_absolute():
                raise BundleError(f'refusing pattern {pattern}: outside the repo')
            matches = [p for p in repo.glob(pattern) if p.is_file() and '__pycache__' not in p.parts]
            if not matches:
                raise BundleError(f'manifest line matches no file: {pattern}')
            for p in matches:
                rel = p.relative_to(repo).as_posix()
                check_path(rel, repo)
                files.add(rel)
    return sorted(files)


def build_zip(files, repo=REPO):
    """Return the zip's bytes. Same files, same bytes."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for rel in files:
            info = zipfile.ZipInfo(rel, date_time=ZIP_DATE)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            info.create_system = 3  # unix, so the mode above is read back the same everywhere
            zf.writestr(info, (Path(repo) / rel).read_bytes())
    return buf.getvalue()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--repo', type=Path, default=REPO)
    ap.add_argument('--manifest', type=Path, default=MANIFEST)
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT, help='output folder (default web/public/py)')
    ap.add_argument('--list', action='store_true', help='print the selected files and exit')
    a = ap.parse_args(argv)
    try:
        files = collect(a.repo, a.manifest)
    except BundleError as e:
        print(f'build_bundle: {e}', file=sys.stderr)
        return 1
    if a.list:
        print('\n'.join(files))
        return 0
    data = build_zip(files, a.repo)
    digest = hashlib.sha256(data).hexdigest()
    raw = sum((Path(a.repo) / f).stat().st_size for f in files)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / ZIP_NAME).write_bytes(data)
    info = {'zip': ZIP_NAME, 'sha256': digest, 'files': len(files), 'bytes': len(data), 'raw_bytes': raw}
    (a.out / INFO_NAME).write_text(json.dumps(info, indent=1) + '\n')
    print(f'{a.out / ZIP_NAME}: {len(files)} files, {raw / 1e6:.2f} MB raw, {len(data) / 1e6:.2f} MB zipped, '
          f'sha256 {digest[:12]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
