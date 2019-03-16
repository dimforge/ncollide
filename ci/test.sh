#! /bin/bash

set -ev

cd $TRAVIS_BUILD_DIR/build/ncollide2d
cargo test
cd $TRAVIS_BUILD_DIR/build/ncollide3d
cargo test
