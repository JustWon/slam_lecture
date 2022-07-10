#ifndef UTILS_HPP
#define UTILS_HPP

#include <stdio.h>

#include "struct.hpp"

void writeLidarPts(std::vector<LidarPoint> &input, const char* fileName);
void readLidarPts(const char* fileName, std::vector<LidarPoint> &output);
template<typename T> void write_pod(std::ofstream& out, T& t);
template<typename T> void read_pod(std::ifstream& in, T& t);
template<typename T> void read_pod_vector(std::ifstream& in, std::vector<T>& vect);
template<typename T> void write_pod_vector(std::ofstream& out, std::vector<T>& vect);

#endif // UTILS_HPP