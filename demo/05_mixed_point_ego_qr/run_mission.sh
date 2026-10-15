#!/usr/bin/env bash
# contest2 Demo05 primary 3rd-terminal entry. Equivalent to run_demo7.sh.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$D/run_demo7.sh" "$@"

# this is a test
