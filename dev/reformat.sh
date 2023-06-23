#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $(dirname $SCRIPT_DIR)

find . -iname *.h -o -iname *.cpp | xargs clang-format -i
python3 -m black .
find . -iname *CMakeLists.txt* | xargs python3 -m cmakelang.format -i

popd
