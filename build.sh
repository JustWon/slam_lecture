#!/bin/bash

build_function ()
{
    sudo rm -rf build
    mkdir build
    cd build
    cmake ../
    make
}

ROOT_DIR=$PWD
echo $ROOT_DIR

# build visual slam examples
cd $ROOT_DIR
cd 3DPointCameraProjection
build_function

cd $ROOT_DIR
cd BundleAdjustment
build_function

cd $ROOT_DIR
cd Fundamental&EssentialMatrix
build_function

cd $ROOT_DIR
cd GraphOptimization
build_function

cd $ROOT_DIR
cd KalmanFilter
build_function

cd $ROOT_DIR
cd Keypoint&Descriptor
build_function

cd $ROOT_DIR
cd LeastSquares
build_function

cd $ROOT_DIR
cd LoopClosureDetection
build_function

cd $ROOT_DIR
cd PerspectiveNPoints
build_function

cd $ROOT_DIR
cd Triangulation 
build_function