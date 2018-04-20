#pragma once
#include "CommonStructures.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// This class is implemented for plane constrained warp
// Specifically, E = Edata + Esim + Ehomo + Ehc
// For the detail, please refer to our paper

class PCWarpSolverAdaptive
{
public:
    PCWarpSolverAdaptive(void);
    ~PCWarpSolverAdaptive(void);

    void performWarping(
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

        const double                                            dWdata = 1,                 // the weights of four components of the full energy
        const double                                            dWsim  = 1,
        const double                                            dWh    = 1e5,			    // geometry term
        const double                                            dWhc   = 1e5,               // homography term
        
        const double                                            dThreshold	 = 1e-6,        // the threshold to stop the iteration
        const int                                               iterationNum = 20           // the maximum iteration number 
        );

private:
    void printStr(std::string str);

    void initializeResultVector(
    const std::vector<Vertex>                               &vDesVertices,
    const std::vector<int>                                  &vSPIdx,
    const std::vector<cv::Mat>                              &vHomo,
    Eigen::VectorXd                                         &vResult
    );

    void constructResult(
    const Eigen::VectorXd                                   &vResult,
    std::vector<Vertex>                                     &vDesVertices,
    const std::vector<int>                                  &vSPIdx,
    std::vector<cv::Mat>                                    &vHomo
    );

    void fillInJacobian(
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
    );

    void fillInResidual(
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
    );

    void fillInResidualAccordingtoSolution(
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
    );

    void computeDelta( 
            const Eigen::SparseMatrix<double>               &mJacobian,  
            const Eigen::VectorXd                           &vResidual, 
            Eigen::VectorXd                                 &vDelta,
            int                                             vertexNum
           );

    void getSamplePoints(const std::vector<std::vector<Eigen::Vector2i> >        &vSP, 
                     const int                                               nSampleNum,
                     std::vector<std::vector<Eigen::Vector2i> >              &vSPSample);

    void convert2PixelLabel(const std::vector<std::vector<Eigen::Vector2i> >        &vSP,
                        Eigen::MatrixXd &pixelLabel);

    void findVertexLabel(const Eigen::MatrixXd &pixelLabel,
                     std::vector<Vertex> &vVertices);

    double computeEnergy(const Eigen::VectorXd &vResidual);

    double subSolve(
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
        );

    double m_dAdaptiveWeight;

};