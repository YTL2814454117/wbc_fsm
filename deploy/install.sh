#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ $# -lt 1 ]; then
    echo "Usage: sudo bash ${0} /path/to/qianer_g1_<version>.tar.gz" >&2
    exit 1
fi

exec bash "${SCRIPT_DIR}/upgrade.sh" --install "$1"
