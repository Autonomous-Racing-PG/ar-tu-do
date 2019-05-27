FOUND_PROBLEMS=false

# Check C++ code
for FILE in $(find "./" -path './src/external_packages' -prune -o \( -name '*.h' -or -name '*.cpp' \) -print)
do
    if clang-format-3.8 -i -style=file -output-replacements-xml $FILE | grep -c "<replacement " > /dev/null ; then
        echo "Formatting problem in:" $FILE
        FOUND_PROBLEMS=true
    fi
done

# Check python code
PYTHON_DIFF=$(autopep8 --diff --recursive --aggressive --aggressive .)

if echo $PYTHON_DIFF | grep -c +++ > /dev/null ; then
    echo "Found formatting problems in the python code."
    echo "Suggested changes:"
    echo "$PYTHON_DIFF"
    FOUND_PROBLEMS=true
fi


if $FOUND_PROBLEMS == true ; then
    echo "Cancelling the travis build due to badly formatted code."
    echo "Please run ./scripts/format/format-src.sh"
    exit 1
else
    exit 0
fi
