#!/bin/bash
cd "$(git rev-parse --show-toplevel)"

# Clang Format Pre-Commit Hook
scripts/hooks/pre-commit/clang-format-staged-files.sh

# Python Format Pre-Commit Hook
# TODO
