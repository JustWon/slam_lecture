#!/bin/bash

install_function ()
{
    sudo rm -rf build
    mkdir build
    cd build
    cmake ../
    make
    sudo make install
}

ROOT_DIR=$PWD
echo $ROOT_DIR

# install prerequisites
sudo apt install git -y
sudo apt install cmake -y
sudo apt install libopencv-dev -y
sudo apt install libeigen3-dev -y
sudo apt install libgoogle-glog-dev -y 
sudo apt install libsuitesparse-dev -y

# build 3rdparties
cd "$ROOT_DIR/3rdparty/googletest"
install_function

cd "$ROOT_DIR/3rdparty/ceres-solver"
install_function

cd "$ROOT_DIR/3rdparty/g2o"
install_function

cd "$ROOT_DIR/3rdparty/Sophus"
install_function

cd "$ROOT_DIR/3rdparty/DBoW3"
install_function