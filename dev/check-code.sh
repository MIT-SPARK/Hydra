#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd $(dirname $SCRIPT_DIR)

find ./python -iname '*.py' | xargs python3 -m flake8

popd
