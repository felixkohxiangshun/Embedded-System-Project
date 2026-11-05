#!/bin/sh
set -eu
repo_root=$(CDPATH= cd "$(dirname "$0")/../../.." && pwd)
make -C "$repo_root/build_make" -j8 \
    SMP=0 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=0 TEST=1 \
    'GCC=arm-none-eabi-gcc -DWIFI_DIAGNOSTIC_SCAN=1' \
    PROFILE_ID=smp0-usb_cdc-wificyw43-netlwip-mqtt0-crash0-test1-scan1 \
    EXE_FILE=mtk3pico_smp0_usb_cdc_wifi_lwip_scan_test
