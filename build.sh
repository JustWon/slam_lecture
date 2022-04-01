#!/bin/bash

build_function ()
{
    sudo rm -rf build
    mkdir build
    cd build
    cmake ../
    make -j1
}

ROOT_DIR=$PWD
echo $ROOT_DIR

# build rgbd slam examples
cd "$ROOT_DIR/3DPointCameraProjection"
build_function

cd "$ROOT_DIR/3DPointCloudProcessing"
build_function

cd "$ROOT_DIR/3DPointCloudRegistration"
build_function

cd "$ROOT_DIR/BundleAdjustment"
build_function

cd "$ROOT_DIR/GraphOptimization"
build_function

cd "$ROOT_DIR/KalmanFilter"
build_function

cd "$ROOT_DIR/Keypoint&Descriptor"
build_function

cd "$ROOT_DIR/LeastSquares"
build_function

cd "$ROOT_DIR/LoopClosureDetection"
build_function

cd "$ROOT_DIR/Perspective3Points"
build_function

cd "$ROOT_DIR/Triangulation"
build_function
