#!/usr/bin/env sh
set -eu
cd /workspace
python -m unittest discover -s tests/vision -p 'test_*.py' -v
