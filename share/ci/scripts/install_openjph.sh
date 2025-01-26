#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright Contributors to the OpenColorIO Project.

set -ex

TAG="$1"

if [[ $OSTYPE == "msys" ]]; then
    SUDO=""
else
    SUDO="sudo"
fi

git clone -b add-export https://github.com/palemieux/OpenJPH.git
cd OpenJPH

git checkout ${TAG}

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
$SUDO cmake --install .

cd ../..
rm -rf OpenJPH
