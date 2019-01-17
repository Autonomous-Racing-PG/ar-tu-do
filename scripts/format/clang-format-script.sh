#!/bin/bash
for DIRECTORY in $1
do
    echo "Formatting code under $DIRECTORY/"
    find "$DIRECTORY" -path $DIRECTORY'src/external_packages' -prune -o \( -name '*.h' -or -name '*.cpp' \) -print | xargs clang-format -i -style=$2
done
