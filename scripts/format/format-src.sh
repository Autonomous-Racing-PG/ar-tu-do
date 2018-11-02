#!/bin/bash
cd "$(dirname ${BASH_SOURCE[0]})"

./clang-format-script.sh ../../ros_ws/ file

autopep8 --in-place --recursive --aggressive --aggressive ../../ros_ws/
