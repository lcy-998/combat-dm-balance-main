#!/bin/sh
# All tests now use the sole supported015 interface; no output-version macro.
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec sh "$repo/tests/run_rl_host_tests.sh"
