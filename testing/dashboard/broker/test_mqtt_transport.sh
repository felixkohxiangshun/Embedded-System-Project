#!/bin/sh
set -eu
repo_root=$(CDPATH= cd "$(dirname "$0")/../../.." && pwd)
sdk_path=${PICO_SDK_PATH:-"$HOME/pico/pico-sdk"}
port="$repo_root/lib/liblwip/sysdepend/pico_rp2040"
test_root="$repo_root/testing/dashboard/broker"
output=$(mktemp -d /tmp/csc11-mqtt-test.XXXXXX)
trap 'rm -rf "$output"' EXIT HUP INT TERM
cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    -I"$test_root/stub" -I"$port" -I"$port/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$test_root/mqtt_transport_host_test.c" -o "$output/transport_test"
"$output/transport_test"
