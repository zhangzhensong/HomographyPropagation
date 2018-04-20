#include "Geometry2D.h"
#include "trianglelib.h"

#include <Eigen/Dense>
#include <iostream>

Geometry2D::Geometry2D(void)
{
}


Geometry2D::~Geometry2D(void)
{
}

void Geometry2D::BarycentricCoord(const std::vector<Eigen::Vector2d> &cageCoords, const Eigen::Vector2d &queryCoord, std::vector<double> &baryCoords)
{
	Eigen::Vector2d kv02 = cageCoords[0] - cageCoords[2];
	Eigen::Vector2d kv12 = cageCoords[1] - cageCoords[2];

	Eigen::Vector2d kvv2 = queryCoord - cageCoords[2];

	double fm00 = kv02.dot(kv02);
	double fm01 = kv02.dot(kv12);
	double fm11 = kv12.dot(kv12);
	double fr0  = kv02.dot(kvv2);
	double fr1  = kv12.dot(kvv2);

	double fdet = fm00*fm11 - fm01*fm01;

	double inv_det = 1.0/fdet;

	baryCoords.push_back((fm11*fr0 - fm01*fr1) * inv_det);
	baryCoords.push_back((fm00*fr1 - fm01*fr0) * inv_det);
	baryCoords.push_back(1.0 - baryCoords[0] - baryCoords[1]);
}

 // Judge whether a point is inside a triangle or not
// Writen by Jason, CUHK, 2015-3-23
bool Geometry2D::isPointInTriangle(Eigen::Vector2d P, Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C)
{
	std::vector<Eigen::Vector2d> tri;
	tri.push_back(A);
	tri.push_back(B);
	tri.push_back(C);

	std::vector<double> wei;

	Geometry2D::BarycentricCoord(tri,P,wei);
	//Geometry2D::mean_value_coordinates(tri,P,wei);

	if(wei[0] >= 0 && wei[0] <= 1 && wei[1] >= 0 && wei[1] <= 1 && 
		wei[0] + wei[1] <= 1)
		return true;
	else
		return false;
	
    /*Eigen::Vector2d v0 = C - A;
    Eigen::Vector2d v1 = B - A;
    Eigen::Vector2d v2 = P - A;

    float dot00 = v0.dot(v0) ;
    float dot01 = v0.dot(v1) ;
    float dot02 = v0.dot(v2) ;
    float dot11 = v1.dot(v1) ;
    float dot12 = v1.dot(v2) ;

    float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;

    float u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return false ;
    }

    float v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return false ;
    }

    return u + v <= 1;*/
}


//////////////////////////////////////////////////////////////////////////////////
// Input :
//      1. polyCoords :  Coordinates of closed polygon in the Counter
//                       clockwise direction. The input is not tested inside.
//
//       2. queryCoord:   the xyCoords of the query Point
// Output:
//       1:  baryCoords: baryCentric Coords of the query Point.
//
// Reference: Mean Value Coordinates for Arbitrary Planar Polygons:
//            Kai Hormann and Michael Floater;
// Written by:
//            Chaman Singh Verma
//            University of Wisconsin at Madison.
//            18th March, 2011.
// Modified by
//            Jason
//            The Chinese University of Hong Kong
//            22nd, March, 2015
/////////////////////////////////////////////////////////////////////////////////
int Geometry2D::mean_value_coordinates( const std::vector<Eigen::Vector2d> &cageCoords, const Eigen::Vector2d &queryCoord, std::vector<double> &baryCoords)
{
    int nSize = (int)cageCoords.size();
    assert( nSize );

    double dx, dy;

    std::vector<Eigen::Vector2d>  s(nSize);
    for( int i = 0; i < nSize; i++)
    {
        dx  =   cageCoords[i][0] - queryCoord[0];
        dy  =   cageCoords[i][1] - queryCoord[1];
        s[i][0]  =   dx;
        s[i][1]  =   dy;
    }

    baryCoords.resize(nSize);
    for( int i = 0; i < nSize; i++)
        baryCoords[i] = 0.0;

    int ip, im;      // (i+1) and (i-1)
    double ri, rp, Ai, Di, dl, mu;  // Distance
    double eps = 10.0*std::numeric_limits<double>::min();

    // First check if any coordinates close to the cage point or
    // lie on the cage boundary. These are special cases.
    for( int i = 0; i < nSize; i++)
    {
        ip = (i+1)%nSize;
        ri = sqrt( s[i][0]*s[i][0] + s[i][1]*s[i][1] );
        Ai = 0.5*(s[i][0]*s[ip][1] - s[ip][0]*s[i][1]);
        Di = s[ip][0]*s[i][0] + s[ip][1]*s[i][1];
        if( ri <= eps)
        {
            baryCoords[i] = 1.0;
            return 0;
        }
        else if( fabs(Ai) <= 0 && Di < 0.0)
        {
            dx = cageCoords[ip][0] - cageCoords[i][0];
            dy = cageCoords[ip][1] - cageCoords[i][1];
            dl = sqrt(dx*dx + dy*dy);
            assert(dl > eps);
            dx = queryCoord[0] - cageCoords[i][0];
            dy = queryCoord[1] - cageCoords[i][1];
            mu = sqrt(dx*dx + dy*dy)/dl;
            assert( mu >= 0.0 && mu <= 1.0);
            baryCoords[i]  = 1.0-mu;
            baryCoords[ip] = mu;
            return 0;
        }
    }

    // Page #12, from the paper
    std::vector<double> tanalpha(nSize); // tan(alpha/2)
    for( int i = 0; i < nSize; i++)
    {
        ip = (i+1)%nSize;
        im = (nSize-1+i)%nSize;
        ri = sqrt( s[i][0]*s[i][0] + s[i][1]*s[i][1] );
        rp = sqrt( s[ip][0]*s[ip][0] + s[ip][1]*s[ip][1] );
        Ai = 0.5*(s[i][0]*s[ip][1] - s[ip][0]*s[i][1]);
        Di = s[ip][0]*s[i][0] + s[ip][1]*s[i][1];
        tanalpha[i] = (ri*rp - Di)/(2.0*Ai);
    }

    // Equation #11, from the paper
    double wi, wsum = 0.0;
    for( int i = 0; i < nSize; i++)
    {
        im = (nSize-1+i)%nSize;
        ri = sqrt( s[i][0]*s[i][0] + s[i][1]*s[i][1] );
        wi = 2.0*( tanalpha[i] + tanalpha[im] )/ri;
        wsum += wi;
        baryCoords[i] = wi;
    }

    if( fabs(wsum ) > 0.0)
    {
        for( int i = 0; i < nSize; i++)
            baryCoords[i] /= wsum;
    }

    return 0;
}

void Geometry2D::TriangulateioToOutput(triangulateio * tio, std::vector<Vertex> & vertices, std::vector<Triangle>& triangles)
{
    for(int i = 0;i<tio->numberofpoints;i++)
    {
        Vertex v((float)tio->pointlist[i*2], (float)tio->pointlist[i*2+1]);
        vertices.push_back(v);
    }

    for(int i = 0;i<tio->numberoftriangles;i++)
    {
        unsigned int tri[3] = { tio->trianglelist[i * 3] , tio->trianglelist[i * 3 + 1], tio->trianglelist[i * 3 + 2] };
        Triangle t;
        for (int j = 0; j < 3; j++)
        {
            t.nVertices[j] = tri[j];
        }
        triangles.push_back(t);
    }
}


// input: 
// 1. tio, should be initialized 
// 2. vPoints
triangulateio* Geometry2D::fillinPoints(triangulateio* tio,  const std::vector<Eigen::Vector2d> & vPoints)
{
    tio->numberofpoints = (int)(vPoints.size());

    if(tio->numberofpoints == 0) return tio;

    tio->pointlist = (REAL1 *) malloc(tio->numberofpoints * 2 * sizeof(REAL1));

    for(int i = 0;i<tio->numberofpoints;i++)
    {
        tio->pointlist[i*2]	 = vPoints[i][0];
        tio->pointlist[i*2+1]= vPoints[i][1];
    }
    return tio;
}

// input: 
// 1. tio, should be initialized 
// 2. vSegments
triangulateio* Geometry2D::fillinSegements(triangulateio* tio, const std::vector<Eigen::Vector2i> & vSegments)
{
    tio->numberofsegments = (int)(vSegments.size());

    if (tio->numberofsegments > 0)
    {
        tio->segmentlist = (int *)malloc(tio->numberofsegments * 2 * sizeof(int));

        for(int i = 0;i<tio->numberofsegments;i++)
        {
            tio->segmentlist[i*2]   = vSegments[i][0];
            tio->segmentlist[i*2+1] = vSegments[i][1];
        }
    }
    return tio;
}

triangulateio * Geometry2D::InputToTriangulateio(const std::vector<Eigen::Vector2d> & vPoints, const std::vector<Eigen::Vector2i> & vSegments)
{
    struct triangulateio * ans;
    Geometry2D::InitTriangulateio(&ans);
    ans = fillinPoints(ans, vPoints);
    ans = fillinSegements(ans, vSegments);

    return ans;
}

void Geometry2D::InitTriangulateio(triangulateio ** pptio)
{
    (*pptio) = (triangulateio*)malloc(sizeof(triangulateio));

    struct triangulateio * tio = (*pptio);

    tio->pointlist = 0;                      
    tio->pointattributelist = 0;             
    tio->pointmarkerlist = 0;                
    tio->numberofpoints = 0;                 
    tio->numberofpointattributes = 0;        

    tio->trianglelist = 0;                   
    tio->triangleattributelist = 0;          
    tio->trianglearealist = 0;               
    tio->neighborlist = 0;                   
    tio->numberoftriangles = 0;              
    tio->numberofcorners = 0;                
    tio->numberoftriangleattributes = 0;     

    tio->segmentlist = 0;                    
    tio->segmentmarkerlist = 0;              
    tio->numberofsegments = 0;               

    tio->holelist = 0;                       
    tio->numberofholes = 0;                  

    tio->regionlist = 0;                     
    tio->numberofregions = 0;                

    tio->edgelist = 0;               
    tio->edgemarkerlist = 0;         
    tio->normlist = 0;               
    tio->numberofedges = 0;          
}

void Geometry2D::FreeTriangulateio(triangulateio ** pptio, bool in)
{
    struct triangulateio * tio = (*pptio);

    if (tio == 0)
    {
        return;
    }

    if(tio->pointlist != 0) 
    {
        free(tio->pointlist);                     
        tio->pointlist = 0;
    }

    if(tio->pointattributelist != 0)
    {
        free(tio->pointattributelist);
        tio->pointattributelist = 0;
    }

    if(tio->pointmarkerlist != 0)
    {
        free(tio->pointmarkerlist);               
        tio->pointmarkerlist = 0;
    }

    if(tio->trianglelist != 0)
    {
        free(tio->trianglelist);           
        tio->trianglelist = 0;
    }

    if(tio->triangleattributelist != 0)
    {
        free(tio->triangleattributelist);          
        tio->triangleattributelist = 0;
    }
    if(tio->trianglearealist != 0)
    {
        free(tio->trianglearealist);               
        tio->trianglearealist = 0;
    }

    if(tio->neighborlist != 0) 
    {
        free(tio->neighborlist);                   
        tio->neighborlist = 0;
    }

    if(tio->segmentlist != 0) 
    {
        free(tio->segmentlist);                    
        tio->segmentlist = 0;
    }
    if(tio->segmentmarkerlist != 0)
    {
        free(tio->segmentmarkerlist);             
        tio->segmentmarkerlist = 0;
    }

    if(in) // only allocalte mem for "in" triangulateio
    {
        std::cout << in << std::endl;
        if(tio->holelist != 0) 
        {
            free(tio->holelist);                      
            tio->holelist = 0;
        }
    }

    if(in) // only allocalte mem for "in" triangulateio
    {
        if(tio->regionlist != 0) 
        {
            free(tio->regionlist);                     
            tio->regionlist = 0;
        }
    }

    if(tio->edgelist != 0) 
    {
        free(tio->edgelist);               
        tio->edgelist = 0;
    }

    if(tio->edgemarkerlist != 0)
    {
        free(tio->edgemarkerlist);        
        tio->edgemarkerlist = 0;
    }

    if(tio->normlist != 0)
    {
        free(tio->normlist);              
        tio->normlist = 0;
    }

    free(*pptio);
    (*pptio) = 0;
}

triangulateio* Geometry2D::ComputeMeshByTriangle(triangulateio * tio, bool isUseConstraintOnly)
{
    struct triangulateio * ans, * vorout;
    Geometry2D::InitTriangulateio(&ans);
    Geometry2D::InitTriangulateio(&vorout);

    if (isUseConstraintOnly)
    {
        triangulate("zpcQY",tio,ans,vorout);
    }
    else
    {
        //triangulate("zpqa500Q",tio,ans,vorout);
        triangulate("zpcqa200",tio,ans,(struct triangulateio *) NULL);  //
        //triangulate("zcpYQ",tio,ans,vorout);  
        //triangulate("zpYQ",tio,ans,vorout);  
    }

    //Geometry2D::FreeTriangulateio(&vorout);
    return ans;
}

void Geometry2D::buildTriangleMesh           (const int                              minx,
                                  const int                              maxx,
                                  const int                              miny,
                                  const int                              maxy,
                                  std::vector<Vertex>                    &vVertices, 
                                  std::vector<Triangle>                  &vTriangles)
{
    Eigen::Vector2d v[4]; // four corners of an image
    v[0] << minx, miny;
    v[1] << maxx, miny;
    v[2] << maxx, maxy;
    v[3] << minx, maxy;

    std::vector<Eigen::Vector2d> vPoints;
    vPoints.clear();

    for (int i = 0; i < 4; i++)
    {
        vPoints.push_back(v[i]);
    }

    std::vector<Eigen::Vector2i> vSegments;
    vSegments.clear();

    for (int i = 0; i < 3; i++)
    {
        vSegments.push_back(Eigen::Vector2i(i, i+1));
    }

    vSegments.push_back(Eigen::Vector2i(3, 0));
    triangulateio *in = Geometry2D::InputToTriangulateio(vPoints, vSegments);
    triangulateio *out = Geometry2D::ComputeMeshByTriangle(in, false);

    vVertices.clear();
    vTriangles.clear();

    // Initialize mesh triangles and the position of the vertices
    Geometry2D::TriangulateioToOutput(out, vVertices, vTriangles);
    Geometry2D::FreeTriangulateio(&in, true);
    Geometry2D::FreeTriangulateio(&out, false);
}

void Geometry2D::buildTriangleMesh (const int nImgWidth, 
                                    const int nImgHeight,
                                    std::vector<Vertex> &vInitialMeshVertices,
                                    std::vector<Triangle> &vTriangles
                                    )
{
    buildTriangleMesh(0, nImgWidth - 1, 0, nImgHeight - 1, vInitialMeshVertices, vTriangles);
}

double Geometry2D::computeTriangleArea(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C)
{
    Eigen::Vector2d a = A - C;
	Eigen::Vector2d b = B - C;

	Eigen::Matrix<double,2,2> D;
	D.col(0) = a;
	D.col(1) = b;

	// according to http://bit.ly/1eTmZsg, the original area has orientation
	// because all vertices are organized counter-clockwise
	return 0.5 * D.determinant();
}