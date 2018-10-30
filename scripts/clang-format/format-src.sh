#!/bin/bash
cd "$(dirname ${BASH_SOURCE[0]})"

./clang-format-script.sh ../../ros_ws/ file

