#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>

#include "utils.hpp"


/* TEMPLATES */




template<typename T> void write_pod(std::ofstream& out, T& t)
{
	out.write(reinterpret_cast<char*>(&t), sizeof(T));
}


template<typename T> void read_pod(std::ifstream& in, T& t)
{
	in.read(reinterpret_cast<char*>(&t), sizeof(T));
}



template<typename T> void read_pod_vector(std::ifstream& in, std::vector<T>& vect)
{
	long size;

	read_pod(in, size);
	for(int i=0; i<size; ++i)
	{
		T t;
		read_pod(in, t);
		vect.push_back(t);
	}
}



template<typename T> void write_pod_vector(std::ofstream& out, std::vector<T>& vect)
{
	long size = vect.size();
	write_pod<long>(out, size);
	for(auto it=vect.begin(); it!=vect.end(); ++it)
	{
		write_pod<T>(out, *it);
	}
}



void writeLidarPts(std::vector<LidarPoint> &input, const char* fileName)
{
	std::ofstream out(fileName);
	write_pod_vector(out, input);
	out.close();
}



void readLidarPts(const char* fileName, std::vector<LidarPoint> &output)
{
	std::ifstream in(fileName);
	read_pod_vector(in, output);
}