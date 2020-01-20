#!/bin/bash

# Stop the script on any error
set -e

# Get path of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR
git submodule update --init --recursive

echo "Installing ESP32 development toolchain"
sudo apt install git wget flex bison gperf python python-pip python-setuptools python-serial python-click python-cryptography python-future python-pyparsing python-pyelftools cmake ninja-build ccache libffi-dev libssl-dev -y
cd $DIR/esp-idf
./install.sh

# Source environment variables
echo "source $DIR/esp-idf/export.sh" >> ~/.bashrc
source ~/.bashrc

echo "Installing ROS pkg requirements"
cd $DIR/../..
rosdep install --from-path src --ignore-src -r -y

echo "Installation successful!"
