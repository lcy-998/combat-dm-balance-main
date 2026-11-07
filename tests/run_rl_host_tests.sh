#!/bin/sh
# 无电机、无STM32下载。仅用主机C编译器验证真实控制代码；RTOS/AI边界使用桩。
set -eu
ulimit -c 0
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
core="$repo/application/chassis"
cc_bin=${CC:-cc}
tmp=$(mktemp -d "${TMPDIR:-/tmp}/motion31-host.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for name in test_rl_motion31 test_rl_spring test_rl_execution test_rl_capture test_rl_shadow_boundary test_rl_supported015 test_rl_center_contract test_rl_height_runtime; do
    set -- "$repo/tests/$name.c" "$core/rl_motion31.c" "$core/rl_control.c"
    case "$name" in
        test_rl_capture) set -- "$@" "$core/rl_capture.c" ;;
        test_rl_shadow_boundary) set -- "-I$repo/tests/host" "$@" "$core/rl_shadow.c" ;;
    esac
    "$cc_bin" -std=c11 -O2 -Wall -Wextra -Werror "-I$core" "$@" -lm -o "$tmp/$name"
    if [ "$name" = test_rl_capture ]; then
        "$tmp/$name" "$tmp/capture_synthetic.bin"
    else
        "$tmp/$name"
    fi
    if [ "$name" = test_rl_shadow_boundary ]; then
        for scenario in 1 2 3; do "$tmp/$name" "$scenario"; done
    fi
    printf 'PASS %s\n' "$name"
done
printf '8/8 host tests passed; no firmware flash, hardware access or real-time certification.\n'
