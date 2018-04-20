#pragma once
#include "CommonStructures.h"

#include <Eigen/Sparse>
#include <opencv2/opencv.hpp>
#include <vector>

// This is a class used for content-preserving warp
// Please refer to the related siggraph paper
// "content preserving warps for 3d video stabilization", siggraph 2010

class WarpSolver
{
public:
	WarpSolver(void);
	~WarpSolver(void);

	void perform_Warping(	
		const std::vector<cv::Point2f>&		vControlFrom, 
		const std::vector<cv::Point2f>&		vControlTo,
		const std::vector<Vertex>&			vVerticesFrom,
		const double    					dLambdaData,
		const double						dLambdaSmooth,
		std::vector<Triangle>&				vTriangles,
		std::vector<FeaturePoint>&			vFeaturePoints,
		std::vector<Vertex>&				vVerticesTo
		);

	void fillinGdata(
		const std::vector<Triangle>&		vTriangleList,
		const std::vector<FeaturePoint>&	vControlHandlers,
		const std::vector<Vertex>&			vTriangleVertices,
		Eigen::SparseMatrix<double>&		Gdata
	);

	void fillinGsmooth(
		const std::vector<Triangle>&		vTriangleList,
		Eigen::SparseMatrix<double>&	    Gsmooth
		);

	void warpAccordingtoGdGs(
		const Eigen::SparseMatrix<double>&		Gdata,
		const Eigen::SparseMatrix<double>&	    Gsmooth,
		const double							dLambdaData,
		const double							dLambdaSmooth,
		const size_t							nControllerNum,  // i.e., # of feature points
		const size_t							nVerticesNum,
		std::vector<Vertex>&					vVerticesTo
		);

    static void fillinControlPoints(
        const std::vector<cv::Point2f>&		vControlFrom, 
        const std::vector<cv::Point2f>&		vControlTo,
        const std::vector<Vertex>&			vVerticesFrom,
        const std::vector<Triangle>&		vTriangles,
        std::vector<FeaturePoint>&			vFeaturePoints);   //i.e., FeaturePoints

    static void fillinControlPoints2(
        const std::vector<Eigen::Vector2d>&		vControlFrom, 
        const std::vector<Eigen::Vector2d>&		vControlTo,
        const std::vector<Vertex>&			vVerticesFrom,
        const std::vector<Triangle>&		vTriangles,
        std::vector<FeaturePoint>&			vFeaturePoints);   //i.e., FeaturePoints

	static void fillinControlPoints3(
        const std::vector<Eigen::Vector2d>&		vControlFrom, 
        const std::vector<Eigen::Vector2d>&		vControlTo,
        const std::vector<Vertex>&			vVerticesFrom,
        const std::vector<Triangle>&		vTriangles,
        std::vector<FeaturePoint>&			vFeaturePoints);   //i.e., FeaturePoints

	static size_t findTriangle( 
        FeaturePoint&						fp, 
        const std::vector<Vertex>&			vTriangleVertices, 
        const std::vector<Triangle>&		vTriangleList
        );

    static void buildTriangleLocalSystem( 
        const std::vector<Vertex>&			vTriangleVertices, 
        std::vector<Triangle>&				vTriangleList
        );
};