#!/bin/sh
# Historical wrapper; tests use the decoder contract selected by this branch.
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec sh "$repo/tests/run_rl_host_tests.sh"
