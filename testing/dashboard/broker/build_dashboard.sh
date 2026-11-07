#!/bin/sh
# Builds the car firmware with the live dashboard link: WiFi, lwIP and the
# binary MQTT bridge (app_program/dashboard/mqtt_bridge.c) in place of the
# demo client lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c.
# Needs config/wifi_secrets.h (copy from wifi_secrets.example.h).
# Output: build_make/mtk3pico_dashboard.uf2.  Single-core by default: in the
# dual-core build the Motor task never ran (2026-11-07). SMP=1 to try it again.
set -eu
repo_root=$(CDPATH= cd "$(dirname "$0")/../../.." && pwd)
smp=${SMP:-0}
make -C "$repo_root/build_make" -j8 \
    SMP="$smp" CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 \
    "GCC=arm-none-eabi-gcc -DDASH_MQTT_BRIDGE=1 ${EXTRA_CFLAGS:-}" \
    MQTT_PORT_OBJS= \
    PROFILE_ID=smp$smp-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0-dashboard1 \
    EXE_FILE=mtk3pico_dashboard
