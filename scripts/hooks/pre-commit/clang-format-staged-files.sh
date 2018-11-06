#!/bin/bash
for FILE in $(git diff-index --cached --name-only HEAD); do
    case ${FILE} in
        *.cpp | *.h | *.hpp)
            clang-format -i ${FILE} -style=file
            git add ${FILE}
            ;;
        *)
            ;;
    esac
done
