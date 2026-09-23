#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 \
  -Ishared_package/Middleware \
  -Ishared_package/Driver/usb_cdc \
  -Ishared_package/Device \
  -Ishared_package/Algorithm \
  main.cpp \
  shared_package/Middleware/Communication_Interface.cpp \
  shared_package/Driver/usb_cdc/usb_cdc.cpp \
  -pthread \
  -o comm_test_threaded

echo "build ok -> ./comm_test_threaded [device]"
