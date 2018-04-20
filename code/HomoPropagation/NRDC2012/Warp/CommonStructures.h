#pragma once

#include <Eigen/Dense>
#include <vector>

struct Vertex
{
	Eigen::Vector2d vPosition;
	int nPlaneIdx;   // each vertex is assigned to a plane, -1 indicates that this vertex does not belong to any plane

	Vertex(){vPosition << 0,0;}

	Vertex(double x, double y)
	{
		vPosition << x,y;
	}

};

struct Triangle
{
	int nPlaneIndex;

	double dInitArea;		// stores the initial area / after initial warp
	double dDeformedArea;	// stores the deformed area / after optimization

	size_t nVertices[3];

	// definition of each vertex in triangle-local coordinate system
	Eigen::Vector2d vTriCoords[3];

	// every triangle has a corresponding homography
	Eigen::MatrixXd H;

	Triangle()
	{
		H = Eigen::MatrixXd::Zero(3,3);
		H(2,2) = 1;
		nPlaneIndex = -1;   // -1 indicates that this triangle does not belong to any plane
		dInitArea = 0;
		dDeformedArea = 0;
	}

	Triangle(const Triangle& t)
	{
		for (int i = 0; i < 3; i++)
		{
			nVertices[i] = t.nVertices[i];
			vTriCoords[i] = t.vTriCoords[i];		
		}
		H = t.H;
		dInitArea = t.dInitArea;
		dDeformedArea = t.dDeformedArea;
		nPlaneIndex = t.nPlaneIndex;
	}
};

struct Constraint
{
	size_t nVertex;							// point index
	Eigen::Vector2d vConstrainedPos;		// the current position of the constraint point

	Constraint(){nVertex = 0; vConstrainedPos = Eigen::Vector2d::Zero();}
	Constraint(size_t nVert, const Eigen::Vector2d &vPos)
	{
		nVertex = nVert;
		vConstrainedPos = vPos;
	}

	bool operator <(const Constraint& c2) const
	{
		return nVertex < c2.nVertex;
	}
};

struct FeaturePoint
{
	size_t nTriangle;					// each feature belong to one triangle, index of triangle
	std::vector<double>	baryCoords;		// Barycentric Coordinates
	Eigen::Vector2d vPosition;
	Eigen::Vector2d vCorrespondingPosition;
	Eigen::Vector2d vTargetPosition;

	FeaturePoint()
	{
		nTriangle = std::numeric_limits<size_t>::max();
		baryCoords.clear();
		vPosition = Eigen::Vector2d::Zero();
		vCorrespondingPosition = Eigen::Vector2d::Zero();
		vTargetPosition = Eigen::Vector2d::Zero();
	}
};