#ifndef NRDC_COMMON_H_
#define NRDC_COMMON_H_

#include <LibIV\libiv.h>
#include <string>

typedef LibIV::Memory::Array::FastArray2D<v3d>		NRDC;
typedef	std::vector<double>							Histogram;
typedef std::vector<Histogram>						Histograms;
typedef std::vector<v2i>							SuperPixel;
typedef std::vector<SuperPixel>						SuperPixels;

struct SpNode
{
	int						sp_id;
	std::vector<int>		neighbors;
	
	double					tmp0;	
	double					tmp1;
	double					tmp2;
	
	std::vector<SpNode*>	child_nodes;		// for shortest path tree
	std::vector<SpNode*>	parent_nodes;
	std::vector<int>		edge_use_info;
};

typedef std::vector<SpNode*>						SpGraph;
typedef LibIV::Memory::Array::FastArray2D<double>	SpChiDiff;

#endif