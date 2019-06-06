#!/bin/bash
cd "$(dirname ${BASH_SOURCE[0]})"

./clang-format-script.sh ../../ros_ws/ file

autopep8 --in-place --recursive --aggressive --aggressive --exclude="../../ros_ws/src/external_packages,../../ros_ws/build,../../ros_ws/devel" ../../ros_ws/
