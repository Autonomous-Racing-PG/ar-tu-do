#!/bin/bash
cd $TRAVIS_BUILD_DIR

###################### REWRITE ##########################

# check for pull-requests
[ "${TRAVIS_PULL_REQUEST}" = "false" ] || \
	skip "Not running Doxygen for pull-requests."

# check for branch name
[ "${TRAVIS_BRANCH}" = "master" ] || \
	skip "Running Doxygen only for updates on 'master' branch (current: ${TRAVIS_BRANCH})."

###################### REWRITE ##########################

# Create a clean working directory for this script.
mkdir docs_repo
cd docs_repo

# Get the current gh-pages branch
git clone -b gh-pages https://git@$GH_REPO_REF
cd $GH_REPO_NAME

##### Configure git.
git config --global push.default simple
git config user.name "Travis CI"
git config user.email "travis@travis-ci.org"

rm -rf *

# META FOO
# TODO: Comment (2018-11-09)
echo "" > .nojekyll

doxygen $DOXYFILE 2>&1 | tee doxygen.log

if [ -d "html" ] && [ -f "html/index.html" ]; then
    git add --all
    git commit -m "Deploy code docs to GitHub Pages Travis build: ${TRAVIS_BUILD_NUMBER}" -m "Commit: ${TRAVIS_COMMIT}"
    git push --force "https://${GH_REPO_TOKEN}@${GH_REPO_REF}" > /dev/null 2>&1
else
    echo 'Error: Something went wrong' >&2
    exit 1
fi

