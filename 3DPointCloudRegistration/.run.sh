#!/bin/bash
./pose_estimation_3d3d ../data/1.png ../data/2.png ../data/1_depth.png ../data/2_depth.png
./iterative_closest_points ../data/room_scan1.pcd ../data/room_scan2.pcd