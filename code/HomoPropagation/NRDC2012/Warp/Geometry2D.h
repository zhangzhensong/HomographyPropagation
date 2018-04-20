#pragma once
#include "trianglelib.h"
#include "CommonStructures.h"

#include <vector>
#include <Eigen/Dense>

class Geometry2D
{
public:
    Geometry2D(void);
    ~Geometry2D(void);
	
	static void BarycentricCoord			(const std::vector<Eigen::Vector2d> &cageCoords, const Eigen::Vector2d &queryCoord, std::vector<double> &baryCoords);
    static int	mean_value_coordinates		(const std::vector<Eigen::Vector2d> &cageCoords, const Eigen::Vector2d &queryCoord, std::vector<double> &baryCoords);
    static bool isPointInTriangle			(Eigen::Vector2d P, Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C);
    static void buildTriangleMesh           (const int nImgWidth, const int nImgHeight, std::vector<Vertex>& vVertices, std::vector<Triangle>& vTriangles);
    static triangulateio*		fillinPoints				(triangulateio* tio,  const std::vector<Eigen::Vector2d> & vPoints);
    static triangulateio*		fillinSegements				(triangulateio* tio, const std::vector<Eigen::Vector2i> & vSegments);
    static triangulateio *		InputToTriangulateio		(const std::vector<Eigen::Vector2d> & vPoints, const std::vector<Eigen::Vector2i> & vSegments);
    static void					InitTriangulateio			(triangulateio ** pptio);
    static void					FreeTriangulateio			(triangulateio ** pptio, bool in = false);
    static triangulateio *		ComputeMeshByTriangle		(triangulateio * tio, bool isUseConstraintOnly  = true);
    static void					TriangulateioToOutput		(triangulateio * tio, std::vector<Vertex> & vertices, std::vector<Triangle>& triangles);
	static void					buildTriangleMesh           (const int                              minx,
                                  const int                              maxx,
                                  const int                              miny,
                                  const int                              maxy,
                                  std::vector<Vertex>                    &vVertices, 
                                  std::vector<Triangle>                  &vTriangles);
    static double				computeTriangleArea			(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C);
};

