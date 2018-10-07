#! /bin/bash

set -ev

cd $TRAVIS_BUILD_DIR/build/ncollide2d
cargo build
cd $TRAVIS_BUILD_DIR/build/ncollide3d
cargo build
