#!/usr/bin/env sh
set -eu
case "${1:-}" in
  amd64|wsl|ubuntu)
    echo "linux/amd64"
    ;;
  orin|thor|arm64)
    echo "linux/arm64"
    ;;
  *)
    echo "usage: $0 {amd64|wsl|ubuntu|orin|thor|arm64}" >&2
    exit 2
    ;;
esac
