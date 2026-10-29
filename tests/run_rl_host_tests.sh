#!/bin/sh
# 无电机、无STM32下载。仅用主机C编译器验证真实控制代码；RTOS/AI边界使用桩。
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
core="$repo/application/chassis"
cc_bin=${CC:-cc}
tmp=$(mktemp -d "${TMPDIR:-/tmp}/motion31-host.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for name in test_rl_motion31 test_rl_spring test_rl_execution test_rl_capture test_rl_shadow_boundary; do
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
    printf 'PASS %s\n' "$name"
done
for side in 0 1; do
    "$cc_bin" -std=c11 -O2 -Wall -Wextra -Werror -DRL_KINEMATICS_HOST -DHIP_IDENT_SIDE="$side" \
        "-I$core" "$repo/tests/test_hip_ident.c" "$core/hip_ident.c" \
        "$core/rl_motion31.c" "$core/rl_control.c" "$repo/application/balance/legalgorithm.c" \
        -lm -o "$tmp/test_hip_ident_$side"
    "$tmp/test_hip_ident_$side"
done
printf '7/7 host tests passed; no firmware flash, hardware access or real-time certification.\n'
