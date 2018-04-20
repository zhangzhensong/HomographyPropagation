#include "PCWarpSolverAdaptive.h"
#include "WarpSolver.h"
#include <cmath>
#include <ctime>
#include <iostream>
#include <string>
#include <algorithm>
#include <map>
#include <Eigen/SparseCholesky>


const bool isPrintInterResult = false;

void PCWarpSolverAdaptive::printStr(std::string str)
{
    if (isPrintInterResult)
    {
        std::cout << str << std::endl;
    }
}

PCWarpSolverAdaptive::PCWarpSolverAdaptive(void)
{
    m_dAdaptiveWeight = 100;   // 1e-10
}


PCWarpSolverAdaptive::~PCWarpSolverAdaptive(void)
{
}

void PCWarpSolverAdaptive::initializeResultVector(
    const std::vector<Vertex>                               &vDesVertices,
    const std::vector<int>                                  &vSPIdx,
    const std::vector<cv::Mat>                              &vHomo,
    Eigen::VectorXd                                         &vResult
    )
{
    assert(vResult.size() == vDesVertices.size() * 2 + vSPIdx.size() * 8);
    size_t nIdx = 0;
    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        const cv::Mat &H = vHomo[vSPIdx[i]];
        for (int m = 0; m < 3; ++m)
        {
            for (int n = 0; n < 3; ++n)
            {
                if (m == 2 && n == 2)
                    continue;
                else
                    vResult[nIdx++] = H.at<double>(m, n);
            }
        }
    }

    for (int i = 0; i < vDesVertices.size(); ++i)
    {
        vResult[nIdx++] = vDesVertices.at(i).vPosition.x();
        vResult[nIdx++] = vDesVertices.at(i).vPosition.y();
    }
}

void PCWarpSolverAdaptive::constructResult(
    const Eigen::VectorXd                                   &vResult,
    std::vector<Vertex>                                     &vDesVertices,
    const std::vector<int>                                  &vSPIdx,
    std::vector<cv::Mat>                                    &vHomo
    )
{
    assert(vResult.size() == vDesVertices.size() * 2 + vSPIdx.size() * 8);
    size_t nIdx = 0;
    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        cv::Mat &H = vHomo[vSPIdx[i]];
        for (int m = 0; m < 3; ++m)
        {
            for (int n = 0; n < 3; ++n)
            {
                if (m == 2 && n == 2)
                    continue;
                else
                    H.at<double>(m, n) = vResult[nIdx++];
            }
        }
    }

    for (int i = 0; i < vDesVertices.size(); ++i)
    {
        vDesVertices.at(i).vPosition.x() = vResult[nIdx++];
        vDesVertices.at(i).vPosition.y() = vResult[nIdx++];
    }
}

void PCWarpSolverAdaptive::fillInJacobian(
    const std::vector<FeaturePoint>                         &vControlPoints,
    const std::vector<Vertex>                               &vSrcVertices,
    const std::vector<Vertex>                               &vDesVertices,
    const std::vector<Triangle>                             &vTriangles, 
    const std::vector<cv::Mat>                              &vHomo0,
    const std::vector<cv::Mat>                              &vHomo,
    std::vector<int>                                        &validHomo,
    const std::vector<int>                                  &vSPIdx,                    // the superpixels that don't have correct homography
    const std::vector<std::vector<Eigen::Vector2i> >        &vSPSample,
    Eigen::SparseMatrix<double>                             &jacobian,
    const double                                            dWdata,                     // the weights of four components of the full energy
    const double                                            dWsim,
    const double                                            dWh,
    const double                                            dWhc
    )                                                                                   // the jacobian of the energy function
{
    std::vector<Eigen::Triplet<double> > vTriplets;
    vTriplets.clear();
    jacobian.setZero();
    
    size_t rowIdx = 0;
    printStr("Begin fill in the data part of the jacobian..");
    
    const size_t vertexOffset = vSPIdx.size() * 8;
    
    for (size_t i = 0; i < vControlPoints.size(); ++i)
    {
        double w0 = vControlPoints[i].baryCoords[0];
        double w1 = vControlPoints[i].baryCoords[1];
        double w2 = vControlPoints[i].baryCoords[2];

        const Triangle & t = vTriangles[vControlPoints[i].nTriangle];
        size_t n0x = t.nVertices[0] * 2 + vertexOffset;
        size_t n0y = n0x + 1;
        size_t n1x = t.nVertices[1] * 2 + vertexOffset;
        size_t n1y = n1x + 1;
        size_t n2x = t.nVertices[2] * 2 + vertexOffset;
        size_t n2y = n2x + 1;

        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0x, w0 * dWdata));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1x, w1 * dWdata));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n2x, w2 * dWdata));
        ++rowIdx;

        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0y, w0 * dWdata));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1y, w1 * dWdata));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n2y, w2 * dWdata));
        ++rowIdx;
    }
    
    printStr("End data part jacobian..");
    assert(rowIdx == vControlPoints.size() * 2);

    printStr("Begin similarity part jacobian..");
    for (size_t i = 0; i < vTriangles.size(); ++i)
    {
        const Triangle &t = vTriangles[i];
        for (size_t j = 0; j < 3; ++j)
        {
            size_t n0 = j;
            size_t n1 = (j + 1) % 3;
            size_t n2 = (j + 2) % 3;

            double x = t.vTriCoords[j].x();
            double y = t.vTriCoords[j].y();

            size_t n0x = vertexOffset + t.nVertices[n0] * 2;
            size_t n0y = n0x + 1;
            size_t n1x = vertexOffset + t.nVertices[n1] * 2;
            size_t n1y = n1x + 1;
            size_t n2x = vertexOffset + t.nVertices[n2] * 2;
            size_t n2y = n2x + 1;

            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0x, (1 - x) * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0y, (-y) * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1x, x * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1y, y * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n2x, (-1) * dWsim));
            ++rowIdx;

            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0x, (y) * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n0y, (1 - x) * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1x, (-y) * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n1y, x * dWsim));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)n2y, (-1) * dWsim));
            ++rowIdx;
        }
    }

   
    printStr("End similarity part jacobian..");
    assert(rowIdx == 2 * vControlPoints.size() + vTriangles.size() * 3 * 2);
    printStr("Begin geometry part jacobian..");

    std::map<int, int> mapSPIdx;
    mapSPIdx.clear();
    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        mapSPIdx[vSPIdx[i]] = i;
    }

    for (size_t i = 0; i < vDesVertices.size(); i++)
    {
        std::vector<int>::const_iterator it = find(vSPIdx.begin(), vSPIdx.end(), vDesVertices[i].nPlaneIdx);

        if (it == vSPIdx.end())
        {
            continue;
        }
        //std::cout << vDesVertices[i].nPlaneIdx << std::endl;

        Eigen::Vector2d vTarget = vDesVertices[i].vPosition;

        assert(vDesVertices[i].nPlaneIdx != -1);
        
        const cv::Mat H = vHomo[vDesVertices[i].nPlaneIdx];

        assert(!H.empty());

        double h1, h2, h3, h4, h5, h6, h7, h8;
        h1 = H.at<double>(0,0); h2 = H.at<double>(0,1); h3 = H.at<double>(0,2);
        h4 = H.at<double>(1,0); h5 = H.at<double>(1,1); h6 = H.at<double>(1,2);
        h7 = H.at<double>(2,0); h8 = H.at<double>(2,1);
        
        assert(abs(H.at<double>(2,2) - 1) < 1e-5);

        Eigen::Vector2d vSource = vSrcVertices[i].vPosition;
        double x = vSource.x();
        double y = vSource.y();
        
        size_t nx = vertexOffset + i * 2;
        size_t ny = nx + 1;
        
        size_t nh1 = mapSPIdx[vDesVertices[i].nPlaneIdx] * 8;//vDesVertices[i].nPlaneIdx * 8;  // bug
        size_t nh2 = nh1 + 1;
        size_t nh3 = nh1 + 2;
        size_t nh4 = nh1 + 3;
        size_t nh5 = nh1 + 4;
        size_t nh6 = nh1 + 5;
        size_t nh7 = nh1 + 6;
        size_t nh8 = nh1 + 7;


        double wweight = dWh;
        if (validHomo[vDesVertices[i].nPlaneIdx] == 0)
        {
            wweight = m_dAdaptiveWeight;
        }


        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nx, 1 * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh1, (-x)/(h7 * x + h8 * y + 1) * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh2, (-y)/(h7 * x + h8 * y + 1) * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh3, (-1)/(h7 * x + h8 * y + 1) * wweight));
        
        double elem = x * (h1 * x + h2 * y + h3) / pow((h7 * x + h8 * y + 1), 2);
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh7, elem * wweight));
        elem = y * (h1 * x + h2 * y + h3) / pow((h7 * x + h8 * y + 1), 2);
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh8, elem * wweight));
        ++rowIdx;

        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)ny, 1 * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh4, (-x)/(h7 * x + h8 * y + 1) * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh5, (-y)/(h7 * x + h8 * y + 1) * wweight));
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh6, (-1)/(h7 * x + h8 * y + 1) * wweight));

        elem = x * (h4 * x + h5 * y + h6) / pow((h7 * x + h8 * y + 1), 2);
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh7, elem * wweight));
        elem = y * (h4 * x + h5 * y + h6) / pow((h7 * x + h8 * y + 1), 2);
        vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh8, elem * wweight));
        ++rowIdx;
    }
    printStr("End geometry part jacobian..");

    //assert(rowIdx == 2 * vControlPoints.size() + vTriangles.size() * 3 * 2 + vDesVertices.size() * 2);
    printStr("Begin plane constrain part jacobian..");
    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        const cv::Mat H = vHomo[vSPIdx[i]];
        const std::vector<Eigen::Vector2i> &samplePts = vSPSample[i];

        double h1, h2, h3, h4, h5, h6, h7, h8;
        h1 = H.at<double>(0,0); h2 = H.at<double>(0,1); h3 = H.at<double>(0,2);
        h4 = H.at<double>(1,0); h5 = H.at<double>(1,1); h6 = H.at<double>(1,2);
        h7 = H.at<double>(2,0); h8 = H.at<double>(2,1);

        size_t nh1 = i * 8;
        size_t nh2 = nh1 + 1;
        size_t nh3 = nh1 + 2;
        size_t nh4 = nh1 + 3;
        size_t nh5 = nh1 + 4;
        size_t nh6 = nh1 + 5;
        size_t nh7 = nh1 + 6;
        size_t nh8 = nh1 + 7;

        for (int j = 0; j < samplePts.size(); ++j)
        {
            Eigen::Vector2i vSource = samplePts[j];
            double x = vSource.x();
            double y = vSource.y();

            double dDiv = h7 * x + h8 * y + 1;
            
            double wweight = dWhc;
            if (validHomo[vSPIdx[i]] == 0)
            {
                wweight = m_dAdaptiveWeight;
            }
            
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh1, (x/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh2, (y/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh3, (1/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh7, (-x * (h1 * x + h2 * y + h3) / dDiv / dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh8, (-y * (h1 * x + h2 * y + h3) / dDiv / dDiv) * wweight));
            ++rowIdx;

            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh4, (x/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh5, (y/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh6, (1/dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh7, (-x * (h1 * x + h2 * y + h3) / dDiv / dDiv) * wweight));
            vTriplets.push_back(Eigen::Triplet<double>((int)rowIdx, (int)nh8, (-y * (h1 * x + h2 * y + h3) / dDiv / dDiv) * wweight));
            ++rowIdx;
        }
    }
    printStr("End fill in the plane constrain part of the jacobian..");
    
    
//assert(rowIdx == 2 * vControlPoints.size() + vTriangles.size() * 3 * 2 + vDesVertices.size() * 2 + vSPIdx.size() * vSPSample[0].size() * 2);

    
    jacobian.setFromTriplets(vTriplets.begin(), vTriplets.end());
}

void PCWarpSolverAdaptive::fillInResidual(
    const std::vector<FeaturePoint>                         &vControlPoints,
    const std::vector<Vertex>                               &vSrcVertices,
    const std::vector<Vertex>                               &vDesVertices,
    const std::vector<Triangle>                             &vTriangles, 
    const std::vector<cv::Mat>                              &vHomo0,
    const std::vector<cv::Mat>                              &vHomo,
    std::vector<int>                                        &validHomo,
    const std::vector<int>                                  &vSPIdx,                    // the superpixels that don't have correct homography
    const std::vector<std::vector<Eigen::Vector2i> >        &vSPSample,
    Eigen::VectorXd                                         &vResidual,
    const double                                            dWdata,                     // the weights of four components of the full energy
    const double                                            dWsim,
    const double                                            dWh,
    const double                                            dWhc
    )                                                                                   // the residual of the energy function
{
    // fill in the residual of data part
    printStr("Begin fill in the residual of data part..");

    size_t idx = 0;
    for (size_t i = 0; i < vControlPoints.size(); ++i)
    {
        double w0 = vControlPoints[i].baryCoords[0];
        double w1 = vControlPoints[i].baryCoords[1];
        double w2 = vControlPoints[i].baryCoords[2];

        const Triangle & t = vTriangles[vControlPoints[i].nTriangle];

        Eigen::Vector2d v0 = vDesVertices[t.nVertices[0]].vPosition;
        Eigen::Vector2d v1 = vDesVertices[t.nVertices[1]].vPosition;
        Eigen::Vector2d v2 = vDesVertices[t.nVertices[2]].vPosition;

        Eigen::Vector2d vTarget = vControlPoints[i].vTargetPosition;
        Eigen::Vector2d vSub = w0 * v0 + w1 * v1 + w2 * v2 - vTarget;

        vResidual[idx++] = dWdata * vSub[0];
        vResidual[idx++] = dWdata * vSub[1];
    }

    printStr("End fill in the residual of data part..");
    printStr("Begin fill in the residual of similarity part..");

    for (size_t i = 0; i < vTriangles.size(); ++i)
    {
        const Triangle &t = vTriangles[i];
        for (size_t j = 0; j < 3; ++j)
        {
            size_t n0 = j;
            size_t n1 = (j + 1) % 3;
            size_t n2 = (j + 2) % 3;

            double x = t.vTriCoords[j].x();
            double y = t.vTriCoords[j].y();

            Eigen::Vector2d v0 = vDesVertices[t.nVertices[n0]].vPosition;
            Eigen::Vector2d v1 = vDesVertices[t.nVertices[n1]].vPosition;
            Eigen::Vector2d v2 = vDesVertices[t.nVertices[n2]].vPosition;

            vResidual[idx++] = ((1 - x) * v0.x() + (-y) * v0.y() + x * v1.x() + y * v1.y() + (-1) * v2.x()) * dWsim;
            vResidual[idx++] = (y * v0.x() + (1 - x) * v0.y() + (-y) * v1.x() + x * v1.y() + (-1) * v2.y()) * dWsim;

        }
    }

    printStr("End fill in the residual of similarity part..");
    printStr("Begin fill in the part of the geometry part of the residual..");
    for (int i = 0; i < vDesVertices.size(); ++i)
    {
        std::vector<int>::const_iterator it = find(vSPIdx.begin(), vSPIdx.end(), vDesVertices[i].nPlaneIdx);

        if (it == vSPIdx.end())
        {
            continue;
        }

        Eigen::Vector2d vTarget = vDesVertices[i].vPosition;
        const cv::Mat H = vHomo[vDesVertices[i].nPlaneIdx];

        Eigen::Vector2d vSource = vSrcVertices[i].vPosition;
        double x = (H.at<double>(0,0) * vSource.x() + H.at<double>(0,1) * vSource.y() + H.at<double>(0,2)) / (H.at<double>(2,0) * vSource.x() + H.at<double>(2,1) * vSource.y() + H.at<double>(2,2));
        double y = (H.at<double>(1,0) * vSource.x() + H.at<double>(1,1) * vSource.y() + H.at<double>(1,2)) / (H.at<double>(2,0) * vSource.x() + H.at<double>(2,1) * vSource.y() + H.at<double>(2,2));


        double wweight = dWh;
        if (validHomo[vDesVertices[i].nPlaneIdx] == 0)
        {
            wweight = m_dAdaptiveWeight;    // 1e-10
        }

        vResidual[idx++] = (vTarget.x() - x) * wweight;
        vResidual[idx++] = (vTarget.y() - y) * wweight;
    }

    printStr("End fill in the geometry part of the residual..");
    printStr("Begin fill in the homography companion part of the residual..");

    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        const cv::Mat H = vHomo[vSPIdx[i]];
        const cv::Mat H0 = vHomo0[vSPIdx[i]];
        const std::vector<Eigen::Vector2i> &samplePts = vSPSample[i];

        for (int j = 0; j < samplePts.size(); ++j)
        {
            Eigen::Vector2i vSource = samplePts[j];
            double x = (H.at<double>(0,0) * vSource.x() + H.at<double>(0,1) * vSource.y() + H.at<double>(0,2)) / (H.at<double>(2,0) * vSource.x() + H.at<double>(2,1) * vSource.y() + H.at<double>(2,2));
            double y = (H.at<double>(1,0) * vSource.x() + H.at<double>(1,1) * vSource.y() + H.at<double>(1,2)) / (H.at<double>(2,0) * vSource.x() + H.at<double>(2,1) * vSource.y() + H.at<double>(2,2));

            double x0 = (H0.at<double>(0,0) * vSource.x() + H0.at<double>(0,1) * vSource.y() + H0.at<double>(0,2)) / (H0.at<double>(2,0) * vSource.x() + H0.at<double>(2,1) * vSource.y() + H0.at<double>(2,2));
            double y0 = (H0.at<double>(1,0) * vSource.x() + H0.at<double>(1,1) * vSource.y() + H0.at<double>(1,2)) / (H0.at<double>(2,0) * vSource.x() + H0.at<double>(2,1) * vSource.y() + H0.at<double>(2,2));


            double wweight = dWhc;
            if (validHomo[vSPIdx[i]] == 0)
            {
                wweight = m_dAdaptiveWeight;    // 1e-10
            }

            vResidual[idx++] = (x - x0) * wweight;
            vResidual[idx++] = (y - y0) * wweight;
        }
    }

    assert(idx == vResidual.size());
}

void PCWarpSolverAdaptive::fillInResidualAccordingtoSolution(
    const Eigen::VectorXd                                   &vSolution,
    const std::vector<FeaturePoint>                         &vControlPoints,
    const std::vector<Vertex>                               &vSrcVertices,
    const std::vector<Triangle>                             &vTriangles, 
    const std::vector<cv::Mat>                              &vHomo0,
    std::vector<int>                                        &validHomo,
    const std::vector<int>                                  &vSPIdx,                    // the superpixels that don't have correct homography
    const std::vector<std::vector<Eigen::Vector2i> >        &vSPSample,
    Eigen::VectorXd                                         &vResidual,
    const double                                            dWdata,                     // the weights of four components of the full energy, output
    const double                                            dWsim,
    const double                                            dWh,
    const double                                            dWhc
    )                                                                                   // the residual of the energy function
{
    // fill in the residual of data part
    printStr("Begin fill in the residual of data part..");

    size_t idx = 0;
    for (size_t i = 0; i < vControlPoints.size(); ++i)
    {
        double w0 = vControlPoints[i].baryCoords[0];
        double w1 = vControlPoints[i].baryCoords[1];
        double w2 = vControlPoints[i].baryCoords[2];

        const Triangle & t = vTriangles[vControlPoints[i].nTriangle];

        Eigen::Vector2d v0(vSolution[vSPIdx.size() * 8 + t.nVertices[0] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[0] * 2 + 1]); //vDesVertices[t.nVertices[0]].vPosition;
        Eigen::Vector2d v1(vSolution[vSPIdx.size() * 8 + t.nVertices[1] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[1] * 2 + 1]); //= vDesVertices[t.nVertices[1]].vPosition;
        Eigen::Vector2d v2(vSolution[vSPIdx.size() * 8 + t.nVertices[2] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[2] * 2 + 1]); //= vDesVertices[t.nVertices[2]].vPosition;

        Eigen::Vector2d vTarget = vControlPoints[i].vTargetPosition;
        Eigen::Vector2d vSub = w0 * v0 + w1 * v1 + w2 * v2 - vTarget;

        vResidual[idx++] = dWdata * vSub[0];
        vResidual[idx++] = dWdata * vSub[1];
    }

    printStr("End fill in the residual of data part..");
    printStr("Begin fill in the residual of similarity part..");

    for (size_t i = 0; i < vTriangles.size(); ++i)
    {
        const Triangle &t = vTriangles[i];
        for (size_t j = 0; j < 3; ++j)
        {
            size_t n0 = j;
            size_t n1 = (j + 1) % 3;
            size_t n2 = (j + 2) % 3;

            double x = t.vTriCoords[j].x();
            double y = t.vTriCoords[j].y();

            Eigen::Vector2d v0(vSolution[vSPIdx.size() * 8 + t.nVertices[n0] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[n0] * 2 + 1]); //= vDesVertices[n0].vPosition;
            Eigen::Vector2d v1(vSolution[vSPIdx.size() * 8 + t.nVertices[n1] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[n1] * 2 + 1]); //= vDesVertices[n1].vPosition;
            Eigen::Vector2d v2(vSolution[vSPIdx.size() * 8 + t.nVertices[n2] * 2], vSolution[vSPIdx.size() * 8 + t.nVertices[n2] * 2 + 1]); //= vDesVertices[n2].vPosition;

            vResidual[idx++] = ((1 - x) * v0.x() + (-y) * v0.y() + x * v1.x() + y * v1.y() + (-1) * v2.x()) * dWsim;
            vResidual[idx++] = (y * v0.x() + (1 - x) * v0.y() + (-y) * v1.x() + x * v1.y() + (-1) * v2.y()) * dWsim;

        }
    }

    printStr("End fill in the residual of similarity part..");
    printStr("Begin fill in the part of the geometry part of the residual..");

    std::map<int, int> mapSPIdx;
    mapSPIdx.clear();
    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        mapSPIdx[vSPIdx[i]] = i;
    }

    for (int i = 0; i < vSrcVertices.size(); ++i)
    {
        std::vector<int>::const_iterator it = find(vSPIdx.begin(), vSPIdx.end(), vSrcVertices[i].nPlaneIdx);

        if (it == vSPIdx.end())
        {
            continue;
        }

        Eigen::Vector2d vTarget(vSolution[vSPIdx.size() * 8 + i * 2], vSolution[vSPIdx.size() * 8 + i * 2 + 1]); //= vDesVertices[i].vPosition;
        double h1 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8];
        double h2 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 1];
        double h3 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 2];
        double h4 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 3];
        double h5 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 4];
        double h6 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 5];
        double h7 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 6];
        double h8 = vSolution[mapSPIdx[vSrcVertices[i].nPlaneIdx] * 8 + 7];

        Eigen::Vector2d vSource = vSrcVertices[i].vPosition;
        double x = (h1 * vSource.x() + h2 * vSource.y() + h3) / (h7 * vSource.x() + h8 * vSource.y() + 1);
        double y = (h4 * vSource.x() + h5 * vSource.y() + h6) / (h7 * vSource.x() + h8 * vSource.y() + 1);



        double wweight = dWh;

        if (validHomo[vSrcVertices[i].nPlaneIdx] == 0)
        {
            wweight = m_dAdaptiveWeight;
        }



        vResidual[idx++] = (vTarget.x() - x) * wweight;
        vResidual[idx++] = (vTarget.y() - y) * wweight;
    }

    printStr("End fill in the geometry part of the residual..");
    printStr("Begin fill in the homography companion part of the residual..");

    for (int i = 0; i < vSPIdx.size(); ++i)
    {
        double h1, h2, h3, h4, h5, h6, h7, h8;
        h1 = vSolution[i * 8];
        h2 = vSolution[i * 8 + 1];
        h3 = vSolution[i * 8 + 2];
        h4 = vSolution[i * 8 + 3];
        h5 = vSolution[i * 8 + 4];
        h6 = vSolution[i * 8 + 5];
        h7 = vSolution[i * 8 + 6];
        h8 = vSolution[i * 8 + 7];

        const cv::Mat H0 = vHomo0[vSPIdx[i]];
        const std::vector<Eigen::Vector2i> &samplePts = vSPSample[i];

        for (int j = 0; j < samplePts.size(); ++j)
        {
            Eigen::Vector2i vSource = samplePts[j];
            double x = (h1 * vSource.x() + h2 * vSource.y() + h3) / (h7 * vSource.x() + h8 * vSource.y() + 1);
            double y = (h4 * vSource.x() + h5 * vSource.y() + h6) / (h7 * vSource.x() + h8 * vSource.y() + 1);

            double x0 = (H0.at<double>(0,0) * vSource.x() + H0.at<double>(0,1) * vSource.y() + H0.at<double>(0,2)) / (H0.at<double>(2,0) * vSource.x() + H0.at<double>(2,1) * vSource.y() + H0.at<double>(2,2));
            double y0 = (H0.at<double>(1,0) * vSource.x() + H0.at<double>(1,1) * vSource.y() + H0.at<double>(1,2)) / (H0.at<double>(2,0) * vSource.x() + H0.at<double>(2,1) * vSource.y() + H0.at<double>(2,2));


            double wweight = dWhc;
            if (validHomo[vSPIdx[i]] == 0)
            {
                wweight = m_dAdaptiveWeight;
            }

            vResidual[idx++] = (x - x0) * wweight;
            vResidual[idx++] = (y - y0) * wweight;
        }
    }

    assert(idx == vResidual.size());
}

void PCWarpSolverAdaptive::computeDelta( 
            const Eigen::SparseMatrix<double>               &mJacobian,  
            const Eigen::VectorXd                           &vResidual, 
            Eigen::VectorXd                                 &vDelta,
            int                                             vertexNum
           )
{
     const double epsilon = 1e-8;

     //Eigen::SparseMatrix<double> jacVertexPart = mJacobian.rightCols(vertexNum * 2);
     //Eigen::SparseMatrix<double> jact = jacVertexPart.transpose();
     //Eigen::SparseMatrix<double> A = jact * jacVertexPart;

     //Eigen::VectorXd B = jact * vResidual;
     //Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solver;
     //solver.compute(A);
     //if (solver.info() != Eigen::Success)
     //{
     //    std::cerr << "Decomposition error!" << std::endl;
     //}

     //Eigen::VectorXd tmpvDelta = solver.solve(B);
     //if (solver.info() != Eigen::Success)
     //{
     //    std::cerr << "Solving error!" << std::endl;
     //}

     //printStr("Finish cholesky decomposition and solving linear system.");

     //tmpvDelta = -1 * tmpvDelta;
     //vDelta.resize(mJacobian.cols());
     //vDelta.setZero();

     //int gap = mJacobian.cols() - tmpvDelta.size();
     //for (int i = 0; i < tmpvDelta.size(); ++i)
     //{
     //    vDelta[i + gap] = tmpvDelta[i];
     //}
     

     Eigen::SparseMatrix<double> mJt = mJacobian.transpose();
     Eigen::SparseMatrix<double> mA = mJt * mJacobian;

     Eigen::VectorXd vB = mJt * vResidual;
     
     Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solver;
     //Eigen::SparseLU<Eigen::SparseMatrix<double> > solver; 
     //Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver; 
     //Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver; 
     //Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver; // slow fail
     //Eigen::SparseQR<Eigen::SparseMatrix<double>,  Eigen::COLAMDOrdering<int>> solver;  // work

     solver.compute(mA);
     if (solver.info() != Eigen::Success)
     {
         std::cerr << "Decomposition error!" << std::endl;
     }

     vDelta = solver.solve(vB);
     if (solver.info() != Eigen::Success)
     {
         std::cerr << "Solving error!" << std::endl;
     }

     printStr("Finish cholesky decomposition and solving linear system.");

     vDelta = -1 * vDelta;
}

void PCWarpSolverAdaptive::getSamplePoints(const std::vector<std::vector<Eigen::Vector2i> >        &vSP, 
                     const int                                               nSampleNum,
                     std::vector<std::vector<Eigen::Vector2i> >              &vSPSample)
{
    vSPSample.clear();
    vSPSample.resize(vSP.size());

    //srand((unsigned)time(0));
    srand(1);

    for (int i = 0; i < vSP.size(); ++i)
    {
        vSPSample[i].clear();
        for (int j = 0; j < nSampleNum; ++j)
        {
            int tmpIdx = rand() % vSP[i].size();
            vSPSample[i].push_back(vSP[i][tmpIdx]);
        }
    }
}

void PCWarpSolverAdaptive::convert2PixelLabel(const std::vector<std::vector<Eigen::Vector2i> >        &vSP,
                        Eigen::MatrixXd &pixelLabel)
{
    for (size_t i = 0; i < vSP.size(); ++i)
    {
        for (size_t j = 0; j < vSP[i].size(); ++j)
        {
            pixelLabel(vSP[i][j].x(),vSP[i][j].y()) = (double)i;
        }
    }
}

void PCWarpSolverAdaptive::findVertexLabel(const Eigen::MatrixXd &pixelLabel,
                     std::vector<Vertex> &vVertices)
{
    for (int i = 0; i < (int)(vVertices.size()); ++i)
    {
        vVertices[i].nPlaneIdx = (int)((pixelLabel((int)(vVertices[i].vPosition.x()), (int)(vVertices[i].vPosition.y()))));
    }
}

double PCWarpSolverAdaptive::computeEnergy(const Eigen::VectorXd &vResidual)
{
    return vResidual.squaredNorm();
}

double PCWarpSolverAdaptive::subSolve(
        const std::vector<FeaturePoint>                         &vControlPoints,
        const std::vector<Vertex>                               &vSrcVertices,
        const std::vector<Triangle>                             &vTriangles, 
        const std::vector<cv::Mat>                              &vHomo0,
        std::vector<int>                                        &validHomo,
        const std::vector<int>                                  &vSPIdx,                    // the superpixels that don't have correct homography
        const std::vector<std::vector<Eigen::Vector2i> >        &vSPSample,
        const double                                            dWdata,                     // the weights of four components of the full energy, output
        const double                                            dWsim,
        const double                                            dWh,
        const double                                            dWhc,
        const Eigen::VectorXd                                   &vResidual,
        const Eigen::VectorXd                                   &dDeltaX,
        const Eigen::VectorXd                                   &vSolution, 
        Eigen::VectorXd                                         &vPreSolution 
        )
{
    double dFunValue = computeEnergy(vResidual);
    double dStepSize = 1;
    const double dMinStepSize = 1e-21;
    bool bRepeat = true;
    int nRunNum = 0;
    double dNewFunValue = 1e20;

    while (bRepeat)
    {
        nRunNum++;
        vPreSolution = vSolution + dStepSize * dDeltaX;
        Eigen::VectorXd vR(vResidual.size());
        vR.setZero();
        fillInResidualAccordingtoSolution(vPreSolution, vControlPoints, vSrcVertices, vTriangles, vHomo0, validHomo, vSPIdx, vSPSample, vR, dWdata, dWsim, dWh, dWhc);
        dNewFunValue = computeEnergy(vR);
        if (dNewFunValue < dFunValue || dStepSize < dMinStepSize)
        {
            std::cout << "new function value OK, iteration number is " << nRunNum << std::endl;
            bRepeat = false;
        }
        else
        {
            dStepSize *= 0.5;
        }
    }

    if (bRepeat || dNewFunValue > dFunValue)
        return 1e20;
    else
        return dNewFunValue;
}

void PCWarpSolverAdaptive::performWarping(
    const int                                               nImgWidth,
    const int                                               nImgHeight,
    std::vector<Vertex>                                     &vSrcVertices,              // triangle mesh, the vertices
    std::vector<Vertex>                                     &vDesVertices,
    std::vector<Triangle>                                   &vTriangles,                // the triangles

    const std::vector<Eigen::Vector2d>                      &vHandles,                  // the handle pixels
    const std::vector<Eigen::Vector2d>                      &vHandlesCorr,              // the corresponding pixels

    const std::vector<std::vector<Eigen::Vector2i> >        &vSP,                       // all superpixels
    const std::vector<cv::Mat>                              &vHomo0,                    // the corresponding initial homographies

    const std::vector<int>                                  &vSPID,                     // the id of the superpixel that needs optimization
    std::vector<cv::Mat>                                    &vHomo,                     // the optimized homographies
    std::vector<int>                                        &validHomo,

    const double                                            dWdata,        // the weights of four components of the full energy
    const double                                            dWsim,
    const double                                            dWh,
    const double                                            dWhc,                       

    const double                                            dThreshold,     // the threshold to stop the iteration
    const int                                               iterationNum        // the maximum iteration number 
    )
{
    assert(vSP.size() == vHomo0.size());
    assert(vHomo0.size() == vHomo.size());

    assert(vSrcVertices.size() > 0);
    if (vDesVertices.size() != vSrcVertices.size())
    {
        vDesVertices.clear();
        vDesVertices.resize(vSrcVertices.size());
        for (int i = 0; i < vSrcVertices.size(); ++i)
        {
            vDesVertices[i].vPosition = vSrcVertices[i].vPosition;
        }
    }




    Eigen::MatrixXd mPixelLabel(nImgWidth, nImgHeight);
    mPixelLabel.setZero();
    convert2PixelLabel(vSP, mPixelLabel);
    findVertexLabel(mPixelLabel, vDesVertices);
    findVertexLabel(mPixelLabel, vSrcVertices);
    

    const int nVarNum = (int)(8 * vSPID.size() + 2 * vSrcVertices.size());   // the variable num
    const int nSampleNum = 10;

    int planeConstrainedVertexNum = 0;
    for (size_t i = 0; i < vSrcVertices.size(); ++i)
    {
        std::vector<int>::const_iterator it = find(vSPID.begin(), vSPID.end(), vSrcVertices[i].nPlaneIdx);

        if (it != vSPID.end())
        {
            ++planeConstrainedVertexNum;
        }
    }

    
    const int nTermNum = (int)(2 * vHandles.size() + vTriangles.size() * 3 * 2 + planeConstrainedVertexNum * 2 + vSPID.size() * nSampleNum * 2);
    //const int nTermNum = 2 * vHandles.size() + vTriangles.size() * 3 * 2;

    std::vector<FeaturePoint> vControlPoints;
    vControlPoints.clear();
    WarpSolver::fillinControlPoints2(vHandles, vHandlesCorr, vSrcVertices, vTriangles, vControlPoints);
    WarpSolver::buildTriangleLocalSystem(vSrcVertices, vTriangles);

    std::vector<std::vector<Eigen::Vector2i> > vSPSample;           // for geometry term
    getSamplePoints(vSP, nSampleNum, vSPSample);

    Eigen::VectorXd vSolution(nVarNum);        // answer
    Eigen::SparseMatrix<double> mJacobian(nTermNum, nVarNum);
    Eigen::VectorXd vResidual(nTermNum);
    Eigen::VectorXd vPreSolution(nVarNum);

    int iterIdx = 0;
    double funVal, newFunVal = 0;

    for (iterIdx = 0; iterIdx < iterationNum; ++iterIdx)
    {
        std::cout << "Iteration #" << iterIdx << std::endl;

        vSolution.setZero();
        mJacobian.setZero();
        vResidual.setZero();
        vPreSolution.setZero();

        initializeResultVector(vSrcVertices, vSPID, vHomo, vSolution);

        fillInResidual(vControlPoints, vSrcVertices, vDesVertices, vTriangles, vHomo0, vHomo, validHomo, vSPID, vSPSample, vResidual, dWdata, dWsim, dWh, dWhc);
        fillInJacobian(vControlPoints, vSrcVertices, vDesVertices, vTriangles, vHomo0, vHomo, validHomo, vSPID, vSPSample, mJacobian, dWdata, dWsim, dWh, dWhc);

        Eigen::VectorXd vDelta;
        computeDelta(mJacobian, vResidual, vDelta, (int)(vSrcVertices.size()));

        funVal = computeEnergy(vResidual);
        newFunVal = subSolve(vControlPoints, vSrcVertices, vTriangles, vHomo0, validHomo, vSPID, vSPSample, dWdata, dWsim, dWh, dWhc, vResidual, vDelta, vSolution, vPreSolution);

        std::cout << "Previous Energy is " << funVal << std::endl;
        std::cout << "Current Engery is " << newFunVal << std::endl;

        if (newFunVal < funVal)
        {
            constructResult(vPreSolution, vDesVertices, vSPID, vHomo);
        }

        if (abs(newFunVal - funVal) < dThreshold || newFunVal > funVal)
        {
            break;
        }
    }

    std::cout << "Iteration: " << iterIdx << std::endl;
    std::cout << "Final energy is: " << (newFunVal < funVal? newFunVal:funVal) << std::endl;
    std::cout << "Final epsilon is: " << abs(newFunVal - funVal) << std::endl;
    
}