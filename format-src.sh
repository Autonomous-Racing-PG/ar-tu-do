#!/bin/bash
cd "$(dirname ${BASH_SOURCE[0]})"

./scripts/clang-format/clang-format-script.sh ros_ws/ file

