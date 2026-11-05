#!/bin/sh
# Host only: production decoder/PD/control/task, no device or model replacement.
set -eu
ulimit -c 0
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
core="$repo/application/chassis"
tmp=$(mktemp -d "${TMPDIR:-/tmp}/motion31-supported015.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
cc_bin=${CC:-cc}
for test in test_rl_supported015 test_rl_supported015_shadow; do
    set -- "$repo/tests/$test.c" "$core/rl_motion31.c" "$core/rl_control.c"
    if [ "$test" = test_rl_supported015_shadow ]; then
        set -- "-I$repo/tests/host" "$@" "$core/rl_shadow.c"
    fi
    "$cc_bin" -std=c11 -O2 -Wall -Wextra -Werror -DRL_MOTION31_OUTPUT_VERSION=2 \
        "-I$core" "$@" -lm -o "$tmp/$test"
    "$tmp/$test"
    if [ "$test" = test_rl_supported015_shadow ]; then
        for scenario in 1 2 3; do "$tmp/$test" "$scenario"; done
    fi
done
