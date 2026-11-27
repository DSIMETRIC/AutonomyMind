#!/usr/bin/env sh
set -eu
cd /workspace
python -m unittest discover -s tests/unit -p 'test_*.py' -v
python -m unittest discover -s tests/integration -p 'test_*.py' -v
python -m unittest discover -s tests/e2e -p 'test_*.py' -v
