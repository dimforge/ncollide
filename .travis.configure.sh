#! /bin/bash

mkdir -p ~/.cargo
WDIR=`pwd`
cat <<EOF > ~/.cargo/config
paths = [ "$WDIR/ncollide_queries",
          "$WDIR/ncollide_entities",
          "$WDIR/ncollide_math",
          "$WDIR/ncollide_pipeline",
          "$WDIR/ncollide_procedural",
          "$WDIR/ncollide_queries",
          "$WDIR/ncollide_transformation",
          "$WDIR/ncollide_utils" ]
