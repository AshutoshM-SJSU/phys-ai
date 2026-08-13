#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import urllib.request
import zipfile
from pathlib import Path


BUNDLE_URL = 'https://www.movingai.com/benchmarks/mapf/mapf-map.zip'
DEFAULT_MAPS = {
    'room-32-32-4.map',
    'maze-32-32-4.map',
    'random-32-32-20.map',
    'random-64-64-20.map',
}


def main() -> None:
    parser = argparse.ArgumentParser(description='Download selected MovingAI MAPF maps.')
    parser.add_argument('maps', nargs='*', default=sorted(DEFAULT_MAPS))
    parser.add_argument('--output', default='maps')
    args = parser.parse_args()

    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    print(f'Downloading {BUNDLE_URL}')
    with urllib.request.urlopen(BUNDLE_URL, timeout=60) as response:
        archive = zipfile.ZipFile(io.BytesIO(response.read()))

    by_basename = {Path(name).name: name for name in archive.namelist() if name.endswith('.map')}
    missing = [name for name in args.maps if name not in by_basename]
    if missing:
        raise SystemExit(f'Maps not found in bundle: {missing}')

    for basename in args.maps:
        data = archive.read(by_basename[basename])
        target = output / basename
        target.write_bytes(data)
        print(target)


if __name__ == '__main__':
    main()
