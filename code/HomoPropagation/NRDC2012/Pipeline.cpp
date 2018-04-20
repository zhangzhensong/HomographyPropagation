#include "Pipeline.h"
#include "nrdc_processing.h"
#include "nrdc_show.h"
#include "nrdc_test.h"


#include "Warp\WarpSolver.h"
#include "Warp\Geometry2D.h"
#include "Warp\WarpRender.h"
#include "Preprocess\Preprocess.h"


#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <fstream>

Pipeline::Pipeline(void)
{
}


Pipeline::~Pipeline(void)
{
}


template <class T>
int getArrayLen(T& array)
{
    return (sizeof(array) / sizeof(array[0]));
}

void ReadSegmentation(std::string filePath, std::vector<int> & segmentation_label)
{
    std::cout << "Read segmentation: " << filePath << std::endl;
     
    std::vector<std::vector<v2i>> SP;
    NRDCProcessing::ReadSP((filePath + "SP.txt").c_str(), SP);

    segmentation_label.resize(SP.size(), 1);

    if (!Preprocess::tellMeIfFileExist(filePath + "mask_car.bmp"))
    {
        return;
    }
    cv::Mat image_car_mask = cv::imread(filePath + "mask_car.bmp",CV_LOAD_IMAGE_GRAYSCALE);

    int imgWidth = image_car_mask.cols;
    int imgHeight = image_car_mask.rows;  

    for (int i = 0; i < SP.size(); ++i)
    {
        int nCntMaskCar = 0;
        int nCntMastBackground = 0;
        for (int j = 0; j < SP[i].size(); ++j)
        {
            if (image_car_mask.at<uchar>(SP[i][j][1], SP[i][j][0]) > 128)
            {
                nCntMaskCar++;
            }
            else
            {
                nCntMastBackground++;
            }
        }

        if (nCntMaskCar > nCntMastBackground)
        {
            segmentation_label[i] = 2;
        }
    } 
}

void Pipeline::TestBidirectional(
    std::string work_dir_path,
    std::string forward_dir_path,
    std::string backward_dir_path)
{
	std::vector<int> segmentation_label_forward;
	std::vector<int> segmentation_label_backward;
    cv::Mat img_src_forward, img_src_backward;
    LibIV::Memory::Array::FastArray2D<v3d> src_dst_corre_forward, src_dst_corre_backward;

    // save the correspondence in file for time saving
    union foo
    { 
        char c[sizeof(double)]; 
        double d;
    }bar;


#ifdef __FORWARD

    NRDCTest::isRunningBackward = false;
    ReadSegmentation(forward_dir_path + "/", segmentation_label_forward);
    NRDCProcessing::ComputeSrcDstCompleteCorre(forward_dir_path,segmentation_label_forward,img_src_forward,src_dst_corre_forward);    
    

	std::ofstream outfile_forward_seg_label(work_dir_path + "/correspondence/segmentation_label_forward.txt");
	for(size_t i = 0;i<segmentation_label_forward.size();i++)
		outfile_forward_seg_label<<segmentation_label_forward[i]<<'\n';
	outfile_forward_seg_label.close();

    std::ofstream outfile_forward_corre(work_dir_path + "/correspondence/forward_corre.txt", std::ios::binary);

    for(int i = 0;i<img_src_forward.rows;i++)
    {
        for(int j = 0;j<img_src_forward.cols;j++)
        {
            v3d cc = src_dst_corre_forward.at(i,j);
            bar.d = cc[0];
            outfile_forward_corre.write(bar.c,sizeof(double));
            bar.d = cc[1];
            outfile_forward_corre.write(bar.c,sizeof(double));
            bar.d = cc[2];
            outfile_forward_corre.write(bar.c,sizeof(double));
        }
    }
    outfile_forward_corre.close();

#endif // __FORWARD


#ifdef __BACKWARD

    NRDCTest::isRunningBackward = true;
    ReadSegmentation(backward_dir_path + "/", segmentation_label_backward);
    NRDCProcessing::ComputeSrcDstCompleteCorre(backward_dir_path,segmentation_label_backward,img_src_backward,src_dst_corre_backward);

	std::ofstream outfile_backward_seg_label(work_dir_path + "/correspondence/segmentation_label_backward.txt");
	for(size_t i = 0;i<segmentation_label_backward.size();i++)
		outfile_backward_seg_label<<segmentation_label_backward[i]<<'\n';
	outfile_backward_seg_label.close();

    std::ofstream outfile_backward_corre(work_dir_path + "/correspondence/backward_corre.txt", std::ios::binary);
    for(int i = 0;i<img_src_backward.rows;i++)
    {
        for(int j = 0;j<img_src_backward.cols;j++)
        {
            v3d cc = src_dst_corre_backward.at(i,j);
            bar.d = cc[0];
            outfile_backward_corre.write(bar.c,sizeof(double));
            bar.d = cc[1];
            outfile_backward_corre.write(bar.c,sizeof(double));
            bar.d = cc[2];
            outfile_backward_corre.write(bar.c,sizeof(double));
        }
    }
    outfile_backward_corre.close();
    
#endif // __BACKWARD

}

void Pipeline::TestReadCorre(
    std::string work_dir_path,
    std::string forward_dir_path,
    std::string backward_dir_path)
{
	cv::Mat img_src_forward  = cv::imread(forward_dir_path + "/a.bmp");

    std::cout << img_src_forward.cols << std::endl;
	
	//std::vector<int> seg_label_forward, seg_label_backward;
	//
	//std::string line;
	std::ifstream infile;//(work_dir_path + "/correspondence/segmentation_label_forward.txt");
	

	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    } bar;

    long long int ss = 0;
    char * buf = NULL;
    char * p = NULL;

	LibIV::Memory::Array::FastArray2D<v3d> corre_forward, corre_backward;


#ifdef __FORWARD

	corre_forward.set(img_src_forward.cols,img_src_forward.rows);
	corre_forward.fill(_v3d_(0,0,0));

	infile.open(work_dir_path + "/correspondence/forward_corre.txt", std::ios::binary);
	if(infile)
	{
		infile.seekg(0, std::ios::end);
		ss = infile.tellg();
		buf = new char[(unsigned int)ss];
		infile.seekg(0, std::ios::beg);
		infile.read(buf,ss);
		p = buf;
	
		for(int i = 0;i<img_src_forward.rows;i++)
		{
			for(int j = 0;j<img_src_forward.cols;j++)
			{
				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_forward.at(i,j)[0] = bar.d;

				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_forward.at(i,j)[1] = bar.d;

				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_forward.at(i,j)[2] = bar.d;
			}
		}
		infile.close();
		delete [] buf;
	}
	else
	{
		return;
	}
    NRDCProcessing::ViewInterpolation(work_dir_path,3,img_src_forward,corre_forward,true);

#endif//__FORWARD

#ifdef __BACKWARD
    cv::Mat img_src_backward = cv::imread(backward_dir_path + "/a.bmp");
    corre_backward.set(img_src_backward.cols,img_src_backward.rows);
    corre_backward.fill(_v3d_(0,0,0));
	infile.open(work_dir_path + "/correspondence/backward_corre.txt", std::ios::binary);
	if(infile)
	{
		infile.seekg(0, std::ios::end);
		ss = infile.tellg();
		buf = new char[(unsigned int)ss];
		infile.seekg(0, std::ios::beg);
		infile.read(buf,ss);
		p = buf;
		
		for(int i = 0;i<img_src_backward.rows;i++)
		{
			for(int j = 0;j<img_src_backward.cols;j++)
			{
				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_backward.at(i,j)[0] = bar.d;

				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_backward.at(i,j)[1] = bar.d;

				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				corre_backward.at(i,j)[2] = bar.d;
			}
		}
		infile.close();
		delete [] buf;
	}
	else
	{
		return;
	}

    //NRDCProcessing::ViewInterpolation(work_dir_path,3,img_src_forward,img_src_backward,
    //    corre_forward,
    //    corre_backward);
    NRDCProcessing::ViewInterpolation(work_dir_path,3,img_src_backward,corre_backward,false);

    #endif // __BACKWARD


}

void Pipeline::TestTriangle()
{
    Eigen::Vector2d v[5]; // four corners of an image
    v[0] << 10, 100;
    v[1] << 100, 10;
    v[2] << 10, 10;
    v[3] << 100, 100;
    v[4] << 80,50;

    std::vector<Eigen::Vector2d> vPoints;
    vPoints.clear();
    std::vector<Eigen::Vector2i> vSegments;
    vSegments.clear();

    int num = sizeof(v)/sizeof(Eigen::Vector2d);

    for (int i = 0; i < num; i++)
    {
        vPoints.push_back(v[i]);
    }

    triangulateio *in = Geometry2D::InputToTriangulateio(vPoints, vSegments);
    triangulateio *out = Geometry2D::ComputeMeshByTriangle(in, true);

    std::vector<Vertex> vVertices;
    std::vector<Triangle> vTriangles;
    vVertices.clear();
    vTriangles.clear();

    // Initialize mesh triangles and the position of the vertices
    Geometry2D::TriangulateioToOutput(out, vVertices, vTriangles);
    Geometry2D::FreeTriangulateio(&in, true);
    Geometry2D::FreeTriangulateio(&out, false);

    cv::Mat output(200,200, CV_8UC1, cv::Scalar(0));
    WarpRender wr;
    wr.drawMesh(vVertices, vTriangles, output);
    cv::imwrite("D:/testTriangle.bmp", output);

}