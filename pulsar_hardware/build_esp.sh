#!/bin/bash
# THIS FILE IS INVOKED THROUGH CMAKE AND SHOULD NOT BE MODIFIED OR CALLED
# MANUALLY. THE INTENDED USE IS FOR 'catkin_make' TO BE CALLED IN THE ROOT
# OF THE WORKSPACE. THIS FILE EXISTS TO ALLOW THE esp-idf TOOLCHAIN TO
# RUN WITHIN THE catkin TOOLCHAIN WITHOUT ISSUES

set -e

OLD_BUILD_PATH=${BUILD_PATH}
OLD_SRC_PATH=${SRC_PATH}
OLD_IDF_PATH=${IDF_PATH}

set -u

export BUILD_PATH="$1"
export SRC_PATH="$2"
export IDF_PATH="$3"

export BUILD_ESP32=1

source "$IDF_PATH/export.sh"

BUILD_PATH="${BUILD_PATH}/pulsar_hardware/esp-idf/build"
rm -rf "$BUILD_PATH"
mkdir -p "$BUILD_PATH"

cd "$BUILD_PATH"

cmake "$SRC_PATH" -DCMAKE_TOOLCHAIN_FILE="$IDF_PATH/tools/cmake/toolchain-esp32.cmake" -DTARGET=esp32 -GNinja
cmake --build .

export BUILD_PATH=${OLD_BUILD_PATH}
export SRC_PATH=${OLD_SRC_PATH}
export IDF_PATH=${OLD_IDF_PATH}
export BUILD_ESP32=""
