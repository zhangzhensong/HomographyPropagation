#include "WarpRender.h"
#include "Geometry2D.h"

#include <omp.h>

WarpRender::WarpRender(void)
{
}


WarpRender::~WarpRender(void)
{

}

cv::Vec3b bilinearIntensity(const cv::Mat& image, const Eigen::Vector2d& vPos)
{
    assert(image.channels() == 3 || image.channels() == 4);

    double x = vPos.x();
    double y = vPos.y();

    int x1 = (int)vPos.x();
    int y1 = (int)vPos.y();

    int x2 = x1 + 1;
    int y2 = y1 + 1;

    x1 = std::max(x1, 1);
    x1 = std::min(x1, image.cols - 1);
    x2 = std::max(x2, 1);
    x2 = std::min(x2, image.cols - 1);

    y1 = std::max(y1, 1);
    y1 = std::min(y1, image.rows - 1);
    y2 = std::max(y2, 1);
    y2 = std::min(y2, image.rows - 1);

    cv::Vec3b Q11, Q12, Q21, Q22;


    Q11 = image.at<cv::Vec3b>(y1, x1);
    Q12 = image.at<cv::Vec3b>(y2, x1);
    Q21 = image.at<cv::Vec3b>(y1, x2);
    Q22 = image.at<cv::Vec3b>(y2, x2);


    cv::Vec3b ret = Q11 * (x2 - x) * (y2 - y) + Q21 * (x - x1) * (y2 - y) +
        Q12 * (x2 - x) * (y - y1) + Q22 * (x - x1) * (y - y1);

    return ret;
}

uchar bilinearIntensitygray(const cv::Mat& gray, const Eigen::Vector2d& vPos)
{
    double x = vPos.x();
    double y = vPos.y();

    int x1 = (int)vPos.x();
    int y1 = (int)vPos.y();

    int x2 = x1 + 1;
    int y2 = y1 + 1;

    double Q11, Q12, Q21, Q22;

    Q11 = (double)gray.at<uchar>(y1, x1);
    Q12 = (double)gray.at<uchar>(y2, x1);
    Q21 = (double)gray.at<uchar>(y1, x2);
    Q22 = (double)gray.at<uchar>(y2, x2);

    double ret = Q11 * (x2 - x) * (y2 - y) + Q21 * (x - x1) * (y2 - y) +
        Q12 * (x2 - x) * (y - y1) + Q22 * (x - x1) * (y - y1);

    return (uchar)ret;
}

cv::Mat drawWarpTriangle(
                      Eigen::Vector2d fromA, 
                      Eigen::Vector2d fromB,
                      Eigen::Vector2d fromC,
                      Eigen::Vector2d toA,
                      Eigen::Vector2d toB,
                      Eigen::Vector2d toC,
                      cv::Mat         fromImage,
                      cv::Mat         toImage)
{
    double minx, miny, maxx, maxy;
    minx = miny = 1e10;
    maxx = maxy = -1e10;

    minx = std::min(toA.x(), toB.x());
    minx = std::min(minx, toC.x());

    miny = std::min(toA.y(), toB.y());
    miny = std::min(miny, toC.y());

    maxx = std::max(toA.x(), toB.x());
    maxx = std::max(maxx, toC.x());

    maxy = std::max(toA.y(), toB.y());
    maxy = std::max(maxy, toC.y());  

    std::vector<Eigen::Vector2d> fromTriangle, toTriangle;
    fromTriangle.clear();
    toTriangle.clear();

    fromTriangle.push_back(fromA);
    fromTriangle.push_back(fromB);
    fromTriangle.push_back(fromC);

    toTriangle.push_back(toA);
    toTriangle.push_back(toB);
    toTriangle.push_back(toC);        

    //std::cout << maxx << ", " << minx << ", " << maxy << ", " << miny << std::endl;

    maxx = std::min(int(maxx + 1), toImage.cols - 1);
    minx = std::max(int(minx - 1), 1);
    maxy = std::min(int(maxy + 1), toImage.rows - 1);
    miny = std::max(int(miny - 1), 1);

    for (int x = minx; x <= maxx; ++x)
    {
        for (int y = miny; y <= maxy; ++y)
        {
            if (Geometry2D::isPointInTriangle(Eigen::Vector2d(x,y), toA, toB, toC))
            {
                //std::cout << "yes!" << std::endl;
                std::vector<double> coeff;
                coeff.clear();

                Eigen::Vector2d curPos(x, y);
                Geometry2D::mean_value_coordinates(toTriangle, curPos, coeff);
                assert(coeff.size() == fromTriangle.size());

                Eigen::Vector2d originPos = coeff.at(0) * fromTriangle.at(0) + coeff.at(1) * fromTriangle.at(1) + coeff.at(2) * fromTriangle.at(2);
                //std::cout << originPos.transpose() << ", " << curPos.transpose() << std::endl;

                if (fromImage.channels() != 1)
                {
                    cv::Vec3b clr = bilinearIntensity(fromImage, originPos);

#pragma omp critical
                    {
                        toImage.at<cv::Vec3b>(y, x) = clr;
                    }
                    
                }
               
            }else
            {
                //std::cout << "no!" << std::endl;
            }

        }
    }

    return toImage;
}


void WarpRender::drawWarpImage(const std::vector<Vertex>           &fromMeshVertices, 
                               const std::vector<Vertex>           &toMeshVertices, 
                               const std::vector<Triangle>         &triangleList,
							   const cv::Mat	&		src_img,
							   cv::Mat			&		dst_img
                               /*const std::string                   originImagePath,
                               const std::string                   targetImagePath*/
                               )
{
    // read origin image from disk
    //cv::Mat originImage = cv::imread(originImagePath);
    //cv::Mat targetImage(src_img.rows, src_img.cols, CV_8UC3, cv::Scalar(0));
	dst_img = cv::Mat(src_img.rows, src_img.cols, CV_8UC3, cv::Scalar(0));

    // render warped image
    #pragma omp parallel for
    for (int i = 0; i < triangleList.size(); ++i)
    {
        // render the i-th triangle
        const size_t * tri = triangleList[i].nVertices;
        drawWarpTriangle(fromMeshVertices[tri[0]].vPosition,
                         fromMeshVertices[tri[1]].vPosition,
                         fromMeshVertices[tri[2]].vPosition,
                         toMeshVertices[tri[0]].vPosition,
                         toMeshVertices[tri[1]].vPosition,
                         toMeshVertices[tri[2]].vPosition,
                         src_img,
                         dst_img 
                         );
    }

    // save to file
    //cv::imwrite(targetImagePath, targetImage);
}

void findRange(const std::vector<Vertex> &vertices, Eigen::Vector2d &xRange, Eigen::Vector2d &yRange)
{
    double minx, miny, maxx, maxy;
    minx = miny = 1e10;
    maxx = maxy = -1e10;

    for (int i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].vPosition.x() < minx)
        {
            minx = vertices[i].vPosition.x();
        }

        if (vertices[i].vPosition.x() > maxx)
        {
            maxx = vertices[i].vPosition.x();
        }

        if (vertices[i].vPosition.y() < miny)
        {
            miny = vertices[i].vPosition.y();
        }

        if (vertices[i].vPosition.y() > maxy)
        {
            maxy = vertices[i].vPosition.y();
        }
    }

    xRange[0] = minx;
    xRange[1] = maxx;
    yRange[0] = miny;
    yRange[1] = maxy;
}

void WarpRender::drawWarpImageOffsetMesh(
                               const std::vector<Vertex>           &fromMeshVertices, 
                               const std::vector<Vertex>           &toMeshVertices, 
                               const std::vector<Triangle>         &triangleList, 
							   const cv::Mat	&		src_img,
							   cv::Mat			&		dst_img
                               /*const std::string                   originImagePath,
                               const std::string                   targetImagePath*/
                               )
{
    // read origin image from disk
    //cv::Mat originImage = cv::imread(originImagePath);
    Eigen::Vector2d xRange, yRange;
    findRange(toMeshVertices, xRange, yRange);

    // offset mesh
    std::vector<Vertex> targetMeshVertices;
    targetMeshVertices.resize(toMeshVertices.size());

    Eigen::Vector2d xymin(xRange[0], yRange[0]);

    #pragma omp parallel for
    for (int i = 0; i < toMeshVertices.size(); ++i)
    {
        targetMeshVertices[i].vPosition = toMeshVertices[i].vPosition - xymin;
    }

    //cv::Mat targetImage(int(yRange[1] - yRange[0] + 1), int(xRange[1] - xRange[0] + 1), CV_8UC3, cv::Scalar(0));
	dst_img = cv::Mat(int(yRange[1] - yRange[0] + 1), int(xRange[1] - xRange[0] + 1), CV_8UC3, cv::Scalar(0));

    // render warped image
    #pragma omp parallel for
    for (int i = 0; i < triangleList.size(); ++i)
    {
        // render the i-th triangle
        const size_t * tri = triangleList[i].nVertices;
        drawWarpTriangle(fromMeshVertices[tri[0]].vPosition,
            fromMeshVertices[tri[1]].vPosition,
            fromMeshVertices[tri[2]].vPosition,
            targetMeshVertices[tri[0]].vPosition,
            targetMeshVertices[tri[1]].vPosition,
            targetMeshVertices[tri[2]].vPosition,
            src_img,
            dst_img
            );
    }

    // save to file
    //cv::imwrite(targetImagePath, targetImage);
}

void WarpRender::drawMesh(
    const std::vector<Vertex>           &MeshVertices, 
    const std::vector<Triangle>         &triangleList, 
    cv::Mat			&		dst_img,
    cv::Scalar                          color
    )
{
    // read origin image from disk
    //cv::Mat originImage = cv::imread(originImagePath);
    Eigen::Vector2d xRange, yRange;
    findRange(MeshVertices, xRange, yRange);

    // offset mesh
    std::vector<Vertex> targetMeshVertices;
    targetMeshVertices.resize(MeshVertices.size());

    Eigen::Vector2d xymin(xRange[0], yRange[0]);

    #pragma omp parallel for
    for (int i = 0; i < MeshVertices.size(); ++i)
    {
        targetMeshVertices[i].vPosition = MeshVertices[i].vPosition;
    }

    //cv::Mat targetImage(int(yRange[1] - yRange[0] + 1), int(xRange[1] - xRange[0] + 1), CV_8UC3, cv::Scalar(0));
    //dst_img = cv::Mat(int(yRange[1] - yRange[0] + 1), int(xRange[1] - xRange[0] + 1), CV_8UC3, cv::Scalar(0));

    // render warped image
    #pragma omp parallel for
    for (int i = 0; i < triangleList.size(); ++i)
    {
        // render the i-th triangle
        const size_t * tri = triangleList[i].nVertices;
        cv::line(dst_img,
            cv::Point((int)targetMeshVertices[tri[0]].vPosition.x(), (int)targetMeshVertices[tri[0]].vPosition.y()),
            cv::Point((int)targetMeshVertices[tri[1]].vPosition.x(), (int)targetMeshVertices[tri[1]].vPosition.y()),
            color
            );
        cv::line(dst_img,
            cv::Point((int)targetMeshVertices[tri[2]].vPosition.x(), (int)targetMeshVertices[tri[2]].vPosition.y()),
            cv::Point((int)targetMeshVertices[tri[1]].vPosition.x(), (int)targetMeshVertices[tri[1]].vPosition.y()),
            color
            );

        cv::line(dst_img,
            cv::Point((int)targetMeshVertices[tri[0]].vPosition.x(), (int)targetMeshVertices[tri[0]].vPosition.y()),
            cv::Point((int)targetMeshVertices[tri[2]].vPosition.x(), (int)targetMeshVertices[tri[2]].vPosition.y()),
            color
            );
    }
}