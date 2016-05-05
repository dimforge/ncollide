#! /bin/bash

mkdir -p ~/.cargo
WDIR=`pwd`
cat <<EOF > ~/.cargo/config
paths = [ "$WDIR/ncollide_math",
          "$WDIR/ncollide_utils",
          "$WDIR/ncollide_geometry",
          "$WDIR/ncollide_pipeline",
          "$WDIR/ncollide_procedural",
          "$WDIR/ncollide_transformation" ]
