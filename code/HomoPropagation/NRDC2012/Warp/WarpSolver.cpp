#include "WarpSolver.h"
#include "Geometry2D.h"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>
#include <cmath>
#include <vector>

WarpSolver::WarpSolver(void)
{
}


WarpSolver::~WarpSolver(void)
{
}

///////////////////////////////////////////////////////////////////
// Function:
//		Calculate which triangle the current vertex is in, and the mean value coordinate of the vertex
// Input:
//		1. fp: the coordinate of the vertex 
//		2. vTriangleVertices: coordintates of all vertices
//		3. vTriangleList: a triangle list, each triangle consists of three indexes to the vertices list
//
// Output:
//		1. the index of the triangle that vertex v lies in
//		2. 
// Written by:
//		Zhensong ZHANG, The Chinese University of Hong Kong, 2015-3-24
///////////////////////////////////////////////////////////////////
size_t WarpSolver::findTriangle(FeaturePoint& fp,								// input/output
                                           const std::vector<Vertex>& vTriangleVertices,	// input
                                           const std::vector<Triangle>& vTriangleList)		// input
{
    Eigen::Vector2d vPos = fp.vPosition;
    for (size_t i = 0; i < vTriangleList.size(); i++)
    {
        const Triangle& t = vTriangleList[i];
        const Eigen::Vector2d& vertA = vTriangleVertices[t.nVertices[0]].vPosition;
        const Eigen::Vector2d& vertB = vTriangleVertices[t.nVertices[1]].vPosition;
        const Eigen::Vector2d& vertC = vTriangleVertices[t.nVertices[2]].vPosition;
        if (Geometry2D::isPointInTriangle(vPos, vertA, vertB, vertC))
        {
            fp.nTriangle = i;
            std::vector<Eigen::Vector2d> cageCoords(3);
            cageCoords[0] << (double)(vertA.x()), (double)(vertA.y());
            cageCoords[1] << (double)(vertB.x()), (double)(vertB.y());
            cageCoords[2] << (double)(vertC.x()), (double)(vertC.y());

            Geometry2D::mean_value_coordinates(cageCoords,Eigen::Vector2d(vPos.x(), vPos.y()),fp.baryCoords);

            return i;
        }
    }
    return std::numeric_limits<size_t>::max();
}


void WarpSolver::fillinControlPoints(const std::vector<cv::Point2f>&			vControlFrom, 
                                                const std::vector<cv::Point2f>&			vControlTo,
                                                const std::vector<Vertex>&				vVerticesFrom,
                                                const std::vector<Triangle>&			vTriangles,
                                                std::vector<FeaturePoint>&				vFeaturePoints)
{
    vFeaturePoints.clear();
    for (size_t i = 0; i < vControlFrom.size(); i++)
    {
        FeaturePoint fp;
        fp.vPosition << vControlFrom[i].x, vControlFrom[i].y;
        fp.vCorrespondingPosition << vControlTo[i].x, vControlTo[i].y;
        fp.vTargetPosition = fp.vCorrespondingPosition;

        size_t nTindex = findTriangle(fp, vVerticesFrom, vTriangles);
        assert(nTindex != std::numeric_limits<size_t>::max());

        vFeaturePoints.push_back(fp);
    }
}

void WarpSolver::fillinControlPoints3(const std::vector<Eigen::Vector2d>&			vControlFrom, 
                                     const std::vector<Eigen::Vector2d>&			vControlTo,
                                     const std::vector<Vertex>&				vVerticesFrom,
                                     const std::vector<Triangle>&			vTriangles,
                                     std::vector<FeaturePoint>&				vFeaturePoints)
{
    vFeaturePoints.clear();
    for (size_t i = 0; i < vControlFrom.size(); i++)
    {
        FeaturePoint fp;
        fp.vPosition = vControlFrom[i];
        fp.vCorrespondingPosition = vControlTo[i];
        fp.vTargetPosition = fp.vCorrespondingPosition;

        size_t nTindex = findTriangle(fp, vVerticesFrom, vTriangles);
        assert(nTindex != std::numeric_limits<size_t>::max());

        vFeaturePoints.push_back(fp);
    }
}

void WarpSolver::fillinControlPoints2(
                                     const std::vector<Eigen::Vector2d>&			vControlFrom, 
                                     const std::vector<Eigen::Vector2d>&			vControlTo,
                                     const std::vector<Vertex>&				vVerticesFrom,
                                     const std::vector<Triangle>&			vTriangles,
                                     std::vector<FeaturePoint>&				vFeaturePoints)
{
	std::vector<int> flag(vControlFrom.size(),0);	

	vFeaturePoints.clear();
	vFeaturePoints.resize(vControlFrom.size());

	for(size_t i = 0;i<vTriangles.size();i++)
	{
		Vertex p0,p1,p2;
		p0 = vVerticesFrom[vTriangles[i].nVertices[0]];
		p1 = vVerticesFrom[vTriangles[i].nVertices[1]];
		p2 = vVerticesFrom[vTriangles[i].nVertices[2]];
		
		double min_x,min_y,max_x,max_y;

		min_x = min_y = 1e20;
		max_x = max_y = -1e20;
	
		if(p0.vPosition.x() < min_x) min_x = p0.vPosition.x();
		if(p1.vPosition.x() < min_x) min_x = p1.vPosition.x();
		if(p2.vPosition.x() < min_x) min_x = p2.vPosition.x();
		
		if(p0.vPosition.x() > max_x) max_x = p0.vPosition.x();
		if(p1.vPosition.x() > max_x) max_x = p1.vPosition.x();
		if(p2.vPosition.x() > max_x) max_x = p2.vPosition.x();

		if(p0.vPosition.y() < min_y) min_y = p0.vPosition.y();
		if(p1.vPosition.y() < min_y) min_y = p1.vPosition.y();
		if(p2.vPosition.y() < min_y) min_y = p2.vPosition.y();

		if(p0.vPosition.y() > max_y) max_y = p0.vPosition.y();
		if(p1.vPosition.y() > max_y) max_y = p1.vPosition.y();
		if(p2.vPosition.y() > max_y) max_y = p2.vPosition.y();

		for(size_t j = 0;j<vControlFrom.size();j++)
		{
			if(flag[j] != 0) 
				continue; // the control point is filled

            Eigen::Vector2d eigen_query = vControlFrom[j];
            //eigen_query = Eigen::Vector2d(0,83);
			
			if(eigen_query.x() < min_x || 
				eigen_query.x() > max_x ||
				eigen_query.y() < min_y ||
				eigen_query.y() > max_y)
				continue; // not in [min,max], thus not in triangle
			
			std::vector<Eigen::Vector2d> eigen_tri;
			std::vector<double> wei;
			
			eigen_tri.push_back(p0.vPosition);
			eigen_tri.push_back(p1.vPosition);
			eigen_tri.push_back(p2.vPosition);
			

			//Geometry2D::BarycentricCoord(eigen_tri,eigen_query,wei);
            Geometry2D::mean_value_coordinates(eigen_tri,eigen_query,wei);

			if(wei[0] >= 0 && wei[0] <= 1 && wei[1] >= 0 && wei[1] <= 1 &&
				wei[0] + wei[1] <= 1) // in triangle
			{
				FeaturePoint fp;
				fp.vPosition = eigen_query;
				fp.vCorrespondingPosition = vControlTo[j];
				fp.vTargetPosition = fp.vCorrespondingPosition;
				fp.nTriangle = i;
				fp.baryCoords.clear();
				fp.baryCoords.push_back(wei[0]);
				fp.baryCoords.push_back(wei[1]);
				fp.baryCoords.push_back(wei[2]);

				vFeaturePoints[j] = fp;

				flag[j] = 1;
			}
		}
	}

	for(size_t i = 0;i<flag.size();i++)
	{
		if(flag[i] == 0)
		{
			std::cout<<"control point "<<i<<" out of mesh"<<std::endl;
		}
	}
}

// build triangle local system, all the relating information is stored in the local triangle
void WarpSolver::buildTriangleLocalSystem(
    const std::vector<Vertex>&		vTriangleVertices,  // input: vertices list
    std::vector<Triangle>&			vTriangleList		// input/output: triangle list
    )
{
    // let's set up triangle-local coordinate system first
    const size_t nTriangles = vTriangleList.size();
    for (size_t i = 0; i < nTriangles; i++)
    {
        Triangle &t = vTriangleList[i];
        for (size_t j = 0; j < 3; j++)
        {
            size_t n0 = j;
            size_t n1 = (j + 1) % 3;
            size_t n2 = (j + 2) % 3;

            Eigen::Vector2d v0 = vTriangleVertices[t.nVertices[n0]].vPosition;
            Eigen::Vector2d v1 = vTriangleVertices[t.nVertices[n1]].vPosition;
            Eigen::Vector2d v2 = vTriangleVertices[t.nVertices[n2]].vPosition;

            // find coordinate system
            Eigen::Vector2d v01(v1 - v0);
            Eigen::Vector2d v01N(v01); 
            v01N.normalize();
            Eigen::Vector2d v01Rot90(v01.y(), -v01.x());	// according to siggraph 05 "As-Rigid-As-Possible Shape Manipulation"
            //Eigen::Vector2d v01Rot90(-v01.y(), v01.x());  // according to siggraph 06 "Image Deformation Using Moving Least Squaress"
            Eigen::Vector2d v01Rot90N(v01Rot90); 
            v01Rot90N.normalize();

            // express v2 in coordinate system
            Eigen::Vector2d vLocal(v2 - v0);
            double fX = vLocal.dot(v01) / v01.squaredNorm();
            double fY = vLocal.dot(v01Rot90) / v01Rot90.squaredNorm();
#if 0
            // check v2 is right or not
            Eigen::Vector2d v2test(v0 + fX * v01 + fY * v01Rot90);
            double fLength  = (v2test - v2).norm();

            std::cout <<  "compare v2: " << fLength << std::endl;
#endif
            t.vTriCoords[j] = Eigen::Vector2d(fX, fY);

        }
    }
}


void WarpSolver::perform_Warping(	
	const std::vector<cv::Point2f>&		vControlFrom, 
	const std::vector<cv::Point2f>&		vControlTo,
	const std::vector<Vertex>&			vVerticesFrom,
	const double						dLambdaData,
	const double    					dLambdaSmooth,
	std::vector<Triangle>&				vTriangles,
	std::vector<FeaturePoint>&			vFeaturePoints,
	std::vector<Vertex>&				vVerticesTo
	)
{
    vVerticesTo.clear();
    vVerticesTo.resize(vVerticesFrom.size());

    //assert(vControlFrom.size() > 2);

	fillinControlPoints(vControlFrom, vControlTo, vVerticesFrom, vTriangles, vFeaturePoints);

	const size_t m = vFeaturePoints.size();					// m stands for the number of controlled points
	const size_t n = vVerticesFrom.size();					// n stands for the number of free points

	Eigen::SparseMatrix<double> Gdata, Gsmooth;		// note that for convenient and Gsmooth can be merged

	Gdata.resize((int)(2 * (m + n)), (int)(2 * (m + n)));
	Gsmooth.resize((int)(2 * n),(int)(2 * n));

	Gdata.setZero();
	Gsmooth.setZero();
  
    fillinGdata(vTriangles, vFeaturePoints, vVerticesFrom, Gdata);
    std::cout << "finish fillinGdata..s"<< std::endl;
    
	// let's set up triangle-local coordinate system first
	const size_t nTriangles = vTriangles.size();
	buildTriangleLocalSystem(vVerticesFrom, vTriangles);


    fillinGsmooth(vTriangles, Gsmooth);
    std::cout << "finish fillinGsmooth.."<< std::endl;

	warpAccordingtoGdGs(Gdata, Gsmooth, dLambdaData, dLambdaSmooth, m, n, vVerticesTo);
}

void WarpSolver::fillinGdata(
	const std::vector<Triangle>&	 vTriangleList,
	const std::vector<FeaturePoint>& vControlHandlers,
	const std::vector<Vertex>&		 vTriangleVertices,
	Eigen::SparseMatrix<double>&	 Gdata
	)
{
	std::vector<Eigen::Triplet<double> > vtriplets;
	vtriplets.clear();

	const size_t m = vControlHandlers.size();
	const size_t n = vTriangleVertices.size();

	for (size_t i = 0; i < m; i++)
	{
		const FeaturePoint& feature = vControlHandlers[i];
		const Triangle& triangle = vTriangleList[feature.nTriangle];

		int n0x = (int)(2 * triangle.nVertices[0]);
		int n0y = (int)(n0x + 1);
		int n1x = (int)(2 * triangle.nVertices[1]);
		int n1y = (int)(n1x + 1);
		int n2x = (int)(2 * triangle.nVertices[2]);
		int n2y = (int)(n2x + 1);
		int nfx = (int)(2 * (n + i));
		int nfy = (int)(nfx + 1);

		// let's fill in Gd
		double x = feature.baryCoords[0];
		double y = feature.baryCoords[1];
		double z = feature.baryCoords[2];
		double s = -feature.vTargetPosition[0];
		double t = -feature.vTargetPosition[1];

		// hence v = (vk0x, vk0y, vk1x, vk1y, vk2x, vk2y, vfx, vfy)', where vfx = vfy = 1
		// A = [x, 0, y, 0, z, 0, s, 0;
		//      0, x, 0, y, 0, z, 0, t]
		// G = A' * A
		// So E = v'*G*v = v'*A'*A*v
		// by deduction, we have G = 
		// [ x*x,         0,		vy*x,         0,		z*x,         0,		  s*x,         0]
		// [   0,	    x*x,		   0,	    y*x,          0,	   z*x,         0,		 t*x]
		// [ x*y,         0,		 y*y,         0,		z*y,         0,		  s*y,         0]
		// [   0,	    x*y,		   0,	    y*y,		  0,	   z*y,         0,		 t*y]
		// [ x*z,         0,		 y*z,         0,		z*z,         0,		  s*z,         0]
		// [   0,	    x*z,		   0,	    y*z,          0,	   z*z,         0,       t*z]
		// [ x*s,         0,		 y*s,         0,		z*s,         0,		  s*s,         0]
		// [   0,	    x*t,           0,	    y*t,          0,	   z*t,         0,       t*t]

		// then we can transfrom G to Gtri, and E = v'*G*v = v'*Gtri*v
		// where Gtri is an upper triangle matrix
		// Gtri = 
		// n0x: [ x*x,         0,		  2*y*x,         0,		   2*z*x,         0,		 2*s*x,         0]
		// n0y: [   0,	     x*x,			  0,	 2*y*x,            0,	  2*z*x,             0,		2*t*x]
		// n1x: [   0,         0,			y*y,         0,		   2*z*y,         0,		 2*s*y,         0]
		// n1y: [   0,	       0,			  0,	   y*y,			   0,	  2*z*y,             0,		2*t*y]
		// n2x: [   0,         0,			  0,         0,			 z*z,         0,		 2*s*z,         0]
		// n2y: [   0,	       0,			  0,	     0,            0,	    z*z,             0,     2*t*z]
		// nfx: [   0,         0,			  0,         0,			   0,         0,		   s*s,         0]
		// nfy: [   0,	       0,             0,	     0,            0,  	      0,             0,       t*t]

		//		 n0x        n0y              n1x        n1y           n2x        n2y            nfx      nfy

		vtriplets.push_back(Eigen::Triplet<double>(n0x, n0x, x*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0x, n1x, 2*y*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0x, n2x, 2*z*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0x, nfx, 2*s*x));

		vtriplets.push_back(Eigen::Triplet<double>(n0y, n0y, x*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0y, n1y, 2*y*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0y, n2y, 2*z*x));
		vtriplets.push_back(Eigen::Triplet<double>(n0y, nfy, 2*t*x));

		vtriplets.push_back(Eigen::Triplet<double>(n1x, n1x, y*y));
		vtriplets.push_back(Eigen::Triplet<double>(n1x, n2x, 2*z*y));
		vtriplets.push_back(Eigen::Triplet<double>(n1x, nfx, 2*s*y));

		vtriplets.push_back(Eigen::Triplet<double>(n1y, n1y, y*y));
		vtriplets.push_back(Eigen::Triplet<double>(n1y, n2y, 2*z*y));
		vtriplets.push_back(Eigen::Triplet<double>(n1y, nfy, 2*t*y));

		vtriplets.push_back(Eigen::Triplet<double>(n2x, n2x, z*z));
		vtriplets.push_back(Eigen::Triplet<double>(n2x, nfx, 2*s*z));

		vtriplets.push_back(Eigen::Triplet<double>(n2y, n2y, z*z));
		vtriplets.push_back(Eigen::Triplet<double>(n2y, nfy, 2*t*z));

		vtriplets.push_back(Eigen::Triplet<double>(nfx, nfx, s*s));
		vtriplets.push_back(Eigen::Triplet<double>(nfy, nfy, t*t));
	}

	Gdata.setFromTriplets(vtriplets.begin(), vtriplets.end());
}

void WarpSolver::fillinGsmooth(
	const std::vector<Triangle>&		vTriangleList,
	Eigen::SparseMatrix<double>&	    Gsmooth
	)
{
	std::vector<Eigen::Triplet<double> > vtriplets;
	vtriplets.clear();

	const size_t nTriangles = vTriangleList.size();
	for (size_t i = 0; i < nTriangles; i++)
	{
		const Triangle & t = vTriangleList[i];
		for (int j= 0; j < 3; j++)
		{
			int n0x = (int)(2 * t.nVertices[j]);
			int n0y = n0x + 1;
			int n1x = (int)(2 * t.nVertices[(j+1)%3]);
			int n1y = n1x + 1;
			int n2x = (int)(2 * t.nVertices[(j+2)%3]);
			int n2y = n2x + 1;
			double x = t.vTriCoords[j].x();
			double y = t.vTriCoords[j].y();

			// according to siggraph 05 "As-Rigid-As-Possible Shape Manipulation"
			// according to Eigen::Vector2d v01Rot90(v01.y(), -v01.x());
			//[ (x - 1)*(x - 1) + y*y,                     0, -2x*(x - 1) - 2y*y,                 2y,   2x - 2,    -2y;
			//                      0, (x - 1)*(x - 1) + y*y,                -2y, -2x*(x - 1) - 2y*y,       2y, 2x - 2;
			//						0,                     0,          x*x + y*y,                  0,      -2x,     2y;
			//        			    0,                     0,                  0,          x*x + y*y,      -2y,    -2x;
			//                      0,                     0,                  0,                  0,        1,      0;
			//                      0,                     0,                  0,                  0,        0,      1]

            // let's explain a little logic here
            // according to Eq(1), let the coordinate of v0, v1 and v2 be (v0x, v0y), (v1x, v1y) and (v2x, v2y), respectively
            // so Eq(1) can be rewrited as
            // (v2x, v2y) = (v0x, v0y) + x * (v1x - v0x, v1y - v0y) + y * (v1y - v0y, v0x - v1x)
            // thus Eq(3) can be rewrited as
            // E = ||((1 - x) * v0x - y * v0y + x * v1x + y * v1y - v2x, y * v0x + (1 - x) * v0y - y * v1x + x * v1y - v2y)||^2
            //   = v' * G * v
            //   = (v0x, v0y, v1x, v1y, v2x, v2y)' * G * (v0x, v0y, v1x, v1y, v2x, v2y)
            // where G = A' * A, and 
            // A = [1 - x,    -y,  x, y, -1,  0;
            //          y, 1 - x, -y, x,  0, -1]
            // So G = 
            //[ (x - 1)*(x - 1) + y*y,                     0, - x*(x - 1) - y*y,                 y, x - 1,    -y;
            //                      0, (x - 1)*(x - 1) + y*y,                -y, - x*(x - 1) - y*y,     y, x - 1;
            //      - x*(x - 1) - y*y,                    -y,         x*x + y*y,         y*x - x*y,    -x,     y;
            //        			    y,     - x*(x - 1) - y*y,         x*y - y*x,         x*x + y*y,    -y,    -x;
            //                  x - 1,                     y,                -x,                -y,     1,     0;
            //                     -y,                 x - 1,                 y,                -x,     0,     1]
            // and E = v' * G * v
            //       = v' * Gtri * v,
            // where Gtri = 
            //[ (x - 1)*(x - 1) + y*y,                     0, -2x*(x - 1) - 2y*y,                 2y,   2x - 2,    -2y;
            //                      0, (x - 1)*(x - 1) + y*y,                -2y, -2x*(x - 1) - 2y*y,       2y, 2x - 2;
            //						0,                     0,          x*x + y*y,                  0,      -2x,     2y;
            //        			    0,                     0,                  0,          x*x + y*y,      -2y,    -2x;
            //                      0,                     0,                  0,                  0,        1,      0;
            //                      0,                     0,                  0,                  0,        0,      1]


			// n0x,n?? elems
			vtriplets.push_back(Eigen::Triplet<double>(n0x, n0x, 1 - 2*x + x*x + y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0x, n1x, 2*x - 2*x*x - 2*y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0x, n1y, 2*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0x, n2x, -2 + 2*x));
			vtriplets.push_back(Eigen::Triplet<double>(n0x, n2y, -2 * y));

			// n0y,n?? elems
			vtriplets.push_back(Eigen::Triplet<double>(n0y, n0y, 1 - 2*x + x*x + y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0y, n1x, -2*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0y, n1y, 2*x - 2*x*x - 2*y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0y, n2x, 2*y));
			vtriplets.push_back(Eigen::Triplet<double>(n0y, n2y, -2 + 2*x));

			// n1x,n?? elems
			vtriplets.push_back(Eigen::Triplet<double>(n1x, n1x, x*x + y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n1x, n2x, -2*x));
			vtriplets.push_back(Eigen::Triplet<double>(n1x, n2y, 2*y));

			// n1y,n?? elems
			vtriplets.push_back(Eigen::Triplet<double>(n1y, n1y, x*x + y*y));
			vtriplets.push_back(Eigen::Triplet<double>(n1y, n2x, -2*y));
			vtriplets.push_back(Eigen::Triplet<double>(n1y, n2y, -2*x));

			// final 2 elems
			vtriplets.push_back(Eigen::Triplet<double>(n2x, n2x, 1));
			vtriplets.push_back(Eigen::Triplet<double>(n2y, n2y, 1));
		}
	}

	Gsmooth.setFromTriplets(vtriplets.begin(), vtriplets.end());
}

void WarpSolver::warpAccordingtoGdGs(
		const Eigen::SparseMatrix<double>&		Gdata,
		const Eigen::SparseMatrix<double>&	    Gsmooth,
		const double							dLambdaData,
		const double							dLambdaSmooth,
		const size_t							nControllerNum,  // i.e., # of feature points
		const size_t							nVerticesNum,
		std::vector<Vertex>&					vVerticesTo
		)
{
	assert(vVerticesTo.size() == nVerticesNum);
	size_t n = nVerticesNum;
	size_t m = nControllerNum;

	Eigen::SparseMatrix<double> mGd00 = Gdata.topLeftCorner((int)(2*n), (int)(2*n));
	Eigen::SparseMatrix<double> mGd01 = Gdata.topRightCorner((int)(2*n),(int)(2*m));
	Eigen::SparseMatrix<double> mGd10 = Gdata.bottomLeftCorner((int)(2*m),(int)(2*n));

	Eigen::SparseMatrix<double> mGd00t = mGd00.transpose();
	Eigen::SparseMatrix<double> Gsmootht = Gsmooth.transpose();

    Eigen::SparseMatrix<double> A = dLambdaData * (mGd00 + mGd00t) + dLambdaSmooth * (Gsmooth + Gsmootht);

	Eigen::SparseMatrix<double> mGd10t =  mGd10.transpose();
	Eigen::SparseMatrix<double> B = mGd01 + mGd10t;
	Eigen::VectorXd vQ(2 * m);
	vQ.setOnes();
	
	Eigen::VectorXd b = B * vQ;
	b = (-1) * b;

	std::cout << "begin solver..compute" << std::endl;

	//Eigen::SparseLU<Eigen::SparseMatrix<double> > solver; 
	//Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver; 
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver; 
	//Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver; // slow fail
	//Eigen::SparseQR<Eigen::SparseMatrix<double>,  Eigen::COLAMDOrdering<int>> solver;
	
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solver;	// work, fast 
	
	//std::cout << A << std::endl;

	solver.compute(A);
	if(solver.info() != Eigen::Success)
	{
		std::cerr << "decomposition failed" << std::endl;
	}

	std::cout << "begin solver..solve" << std::endl;
	Eigen::VectorXd vU = solver.solve(b);

	std::cout << "end solver..solved" << std::endl;
	//std::cout << vU << std::endl;

	double fx;
	double fy; 
	for (size_t i = 0; i < n; i++)
	{
		fx = (double)vU(2 * i);
		fy = (double)vU(2 * i + 1); 

		//std::cout << "Origin: " << vVerticesTo[i].vPosition.transpose() << ",  warped: " << Eigen::Vector2d(fx, fy).transpose() << std::endl;
		vVerticesTo[i].vPosition = Eigen::Vector2d(fx, fy);
	}
	std::cout << "finish warping.." << std::endl;
}
