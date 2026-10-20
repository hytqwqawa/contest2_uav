#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$D/demo10_verify.py"
exec "$D/run_demo9.sh"
