#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y python3.12-venv
fi

if ! command -v python3.12 >/dev/null 2>&1; then
    echo "python3.12 was not found in PATH" >&2
    exit 1
fi

python3.12 -m venv .venv
. .venv/bin/activate
# python -m pip install --upgrade pip
python -m pip install -r requirement.txt

echo "Virtualenv ready: .venv"
