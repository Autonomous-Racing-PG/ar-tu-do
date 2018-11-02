#!/bin/bash
for DIRECTORY in $1
do
    echo "Formatting code under $DIRECTORY/"
    find "$DIRECTORY" \( -name '*.h' -or -name '*.cpp' \) | xargs clang-format -i -style=$2
done
