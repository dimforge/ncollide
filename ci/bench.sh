#! /bin/bash

set -ev

if [ "$RUST_VERSION" == "nightly" ]; then
    cd $TRAVIS_BUILD_DIR/build/ncollide2d
    cargo bench
    cd $TRAVIS_BUILD_DIR/build/ncollide3d
    cargo bench
fi