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
cd "$ROOT_DIR/3DPointCameraProjection"
build_function

cd "$ROOT_DIR/BundleAdjustment"
build_function

cd "$ROOT_DIR/Fundamental&EssentialMatrix"
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

cd "$ROOT_DIR/PerspectiveNPoints"
build_function

cd "$ROOT_DIR/Triangulation"
build_function