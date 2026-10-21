#!/usr/bin/env python3
"""Fetch only the pinned official interface package, without SDK or DDS vendor changes."""

import io
import tarfile
import urllib.request
from pathlib import Path

import yaml


lock = yaml.safe_load(Path('/tmp/dependencies.repos').read_text())
revision = lock['repositories']['unitree_ros2']['version']
url = f'https://codeload.github.com/unitreerobotics/unitree_ros2/tar.gz/{revision}'
with urllib.request.urlopen(url, timeout=120) as response:
    archive = tarfile.open(fileobj=io.BytesIO(response.read()), mode='r:gz')
    prefix = f'unitree_ros2-{revision}/cyclonedds_ws/src/unitree/unitree_go/'
    for member in archive.getmembers():
        if member.isfile() and member.name.startswith(prefix):
            relative = Path(member.name.removeprefix(prefix))
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('Unsafe interface archive path')
            destination = Path('src/unitree_go') / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(archive.extractfile(member).read())
if not Path('src/unitree_go/package.xml').is_file():
    raise RuntimeError('Pinned archive did not contain unitree_go')
