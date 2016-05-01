#!/bin/sh

set -e
# check to see whether local folder needs populating
if [ ! -d "$HOME/local/lib" ]; then
  mkdir -p $HOME/local;
  
  cd $HOME;
  git clone https://github.com/glfw/glfw.git;
  cd glfw;
  git checkout 3.0.3;
  cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$HOME/local;
  make;
  make install;
  
  cd $HOME;
  wget http://www.sfml-dev.org/files/SFML-2.1-sources.zip;
  unzip SFML-2.1-sources.zip;
  cd SFML-2.1;
  cmake -DCMAKE_INSTALL_PREFIX=$HOME/local;
  make;
  make install;
  
  cd $HOME;
  wget http://www.sfml-dev.org/files/CSFML-2.1-sources.zip;
  unzip CSFML-2.1-sources.zip;
  cd CSFML-2.1;
  patch -p1 <$TRAVIS_BUILD_DIR/.travis.payload/CSFML-2.1-CMAKE.patch;
  cmake -DCMAKE_INSTALL_PREFIX=$HOME/local -DSFML_DIR=$HOME/local -DCMAKE_MODULE_PATH=$HOME/local/share/SFML/cmake/Modules;
  make;
  make install;
else
  echo 'Using cached directory.';
fi

# From https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
pushd ~
git clone https://github.com/FFmpeg/FFmpeg.git
cd FFmpeg
git checkout release/3.0
mkdir ~/FFmpeg-build
cd ~/FFmpeg-build
../FFmpeg/configure --disable-ffprobe --disable-ffserver --disable-doc --enable-avresample
make -j
make install
make distclean
popd
