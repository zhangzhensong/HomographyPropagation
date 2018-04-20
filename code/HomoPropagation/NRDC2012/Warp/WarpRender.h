#pragma once
#include "CommonStructures.h"

#include <opencv2/opencv.hpp>
#include <string>

class WarpRender
{
public:
    WarpRender(void);
    ~WarpRender(void);

    void drawWarpImage( const std::vector<Vertex>           &fromMeshVertices, 
                        const std::vector<Vertex>           &toMeshVertices, 
                        const std::vector<Triangle>         &triangleList, 
						const cv::Mat	                    &src_img,
						cv::Mat			                    &dst_img   
						/*const std::string                   originImagePath,
                        const std::string                   targetImagePath*/
                        );

    void drawWarpImageOffsetMesh( 
                        const std::vector<Vertex>           &fromMeshVertices, 
                        const std::vector<Vertex>           &toMeshVertices, 
                        const std::vector<Triangle>         &triangleList, 
						const cv::Mat	&		src_img,
						cv::Mat			&		dst_img
                        /*const std::string                   originImagePath,
                        const std::string                   targetImagePath*/
                        );

    void drawMesh(
        const std::vector<Vertex>           &MeshVertices, 
        const std::vector<Triangle>         &triangleList, 
        cv::Mat			                    &dst_img,
        cv::Scalar                          color  = cv::Scalar(255,255,255)
        );

    cv::Vec3b bilinearIntensity(const cv::Mat& image, const Eigen::Vector2d& vPos);
};

