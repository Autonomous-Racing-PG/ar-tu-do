#!/bin/bash
cd "$(git rev-parse --show-toplevel)"

# Pre-Commit
cp scripts/hooks/pre-commit.sh .git/hooks/pre-commit
