#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
SRC=../src/shared_package

cp "$SRC"/Middleware/Communication_Interface.hpp  shared_package/Middleware/
cp "$SRC"/Middleware/Communication_Interface.cpp  shared_package/Middleware/
cp "$SRC"/Driver/usb_cdc/usb_cdc.hpp              shared_package/Driver/usb_cdc/
cp "$SRC"/Driver/usb_cdc/usb_cdc.cpp              shared_package/Driver/usb_cdc/
cp "$SRC"/Device/motor_base.hpp                   shared_package/Device/
cp "$SRC"/Algorithm/alg_crc.h                     shared_package/Algorithm/

echo "synced from $SRC"
