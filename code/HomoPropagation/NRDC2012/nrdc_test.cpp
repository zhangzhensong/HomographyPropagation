#include "nrdc_test.h"
#include "nrdc_processing.h"
#include "nrdc_show.h"
#include <iostream>
#include <fstream>
#include <opencv2\imgproc\imgproc.hpp>
#include "Warp\WarpSolver.h"
#include "Warp\Geometry2D.h"
#include "Warp\WarpRender.h"


using namespace std;

std::string NRDCTest::folerpath = "E:/test20150716";
bool NRDCTest::isRunningBackward = false;
std::vector<int> NRDCTest::missingRegion;


void NRDCTest::TestHomography()
{
	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

	obj.push_back(cv::Point2d(451, 611));
	scene.push_back(cv::Point2d(316, 725));

	obj.push_back(cv::Point2d(661, 617));
	scene.push_back(cv::Point2d(768, 743));

	obj.push_back(cv::Point2d(605, 545));
	scene.push_back(cv::Point2d(615, 550));

	obj.push_back(cv::Point2d(531, 546));
	scene.push_back(cv::Point2d(521, 548));

	cv::Mat H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );

	H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );
	std::cout << "Small region homography: " << H << std::endl;

	std::vector<cv::Point2d> tgtVec;
	
	tgtVec.resize(obj.size());
	cv::perspectiveTransform( cv::Mat(obj), cv::Mat(tgtVec), H);

	std::string srcfileName = "F:/LastExperiments/Case28/forward/a.bmp";
	std::string tgtfileName = "F:/LastExperiments/Case28/forward/b.bmp";
	
	cv::Mat srcImage = cv::imread(srcfileName);
	cv::Mat tgtImage = cv::imread(tgtfileName);

	const int nCircleSize = 3;

	for (int i = 0; i < tgtVec.size() - 1; i++)
	{
		cv::line(tgtImage, tgtVec[i], tgtVec[i + 1], cv::Scalar(255, 0, 0));
	}

	cv::line(tgtImage, tgtVec[tgtVec.size() - 1], tgtVec[0], cv::Scalar(255, 0, 0));

	for (int i = 0; i < tgtVec.size(); i++)
	{
		cv::circle(tgtImage, tgtVec[i], nCircleSize, cv::Scalar(0, 255, 0));
	}


	for (int i = 0; i < obj.size() - 1; i++)
	{
		cv::line(srcImage, obj[i], obj[i + 1], cv::Scalar(255, 0, 0));
	}

	cv::line(srcImage, obj[obj.size() - 1], obj[0], cv::Scalar(255, 0, 0));


	for (int i = 0; i < obj.size(); i++)
	{
		cv::circle(srcImage, obj[i], nCircleSize, cv::Scalar(0, 255, 0));
		cv::line(srcImage, obj[i], tgtVec[i], cv::Scalar(0, 0, 255));
	}

	obj.clear();

	obj.push_back(cv::Point2d(461, 721));
	obj.push_back(cv::Point2d(760, 660));
	obj.push_back(cv::Point2d(76, 760));
	obj.push_back(cv::Point2d(555, 666));

	tgtVec.resize(obj.size());
	cv::perspectiveTransform( cv::Mat(obj), cv::Mat(tgtVec), H);

	for (int i = 0; i < tgtVec.size(); i++)
	{
		cv::circle(tgtImage, tgtVec[i], nCircleSize, cv::Scalar(0, 255, 0));
		cv::circle(srcImage, obj[i], nCircleSize, cv::Scalar(0, 255, 0));

		std::cout << "source position: " << obj[i] << ", target position: " << tgtVec[i] << std::endl;

		cv::line(srcImage, obj[i], tgtVec[i], cv::Scalar(255, 0, 255));
	}

	cv::imwrite("D:/hellobbb/sourceImage.bmp", srcImage);
	cv::imwrite("D:/hellobbb/transformedImage.bmp", tgtImage);
}

void NRDCTest::TestImgAlignment()
{



}


void NRDCTest::TestAllGround()
{
	int width = 1450;
	int height = 779;

    std::string work_dir = "F:/LastExperiments/Case28/forward/";

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);

	NRDCProcessing::ReadSP((work_dir + "SP.txt").c_str(),SP);	
	NRDCProcessing::ReadNRDC(nrdc,(work_dir + "NRDC.txt").c_str());

	IplImage * img0 = cvLoadImage((work_dir + "a.bmp").c_str());
	IplImage * img1 = cvLoadImage((work_dir + "b.bmp").c_str());

	int sp_idx = 359;
	int sp_idx_check;
	std::vector<v2d> spc;
	cv::Mat H;
	std::vector<int> sp_support;
	std::vector<double> sp_support_diff;


	//NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);	
	
	std::vector<int> seeds;
	seeds.clear();
	seeds.push_back(359);
	//seeds.push_back(370);
	//seeds.push_back(371);
	//seeds.push_back(339);
	//seeds.push_back(342);
	//seeds.push_back(272);
	//seeds.push_back(275);

	//seeds.push_back(217);
	//seeds.push_back(244);

	//seeds.push_back(381);

	//NRDCProcessing::ComputeHomographyForSP(nrdc, SP, seeds, H);

	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

	obj.push_back(cv::Point2d(451, 611));
	scene.push_back(cv::Point2d(316, 725));

	obj.push_back(cv::Point2d(661, 617));
	scene.push_back(cv::Point2d(768, 743));

	obj.push_back(cv::Point2d(531, 546));
	scene.push_back(cv::Point2d(521, 548));

	obj.push_back(cv::Point2d(605, 545));
	scene.push_back(cv::Point2d(615, 550));

	H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );

	std::string fileName = "F:/LastExperiments/Case28/forward/mask/mask_ground.bmp";

	cv::Mat unKnownMaskImg = cv::imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<size_t> maskedSuperpixels;
    findMaskedSuperpixels(unKnownMaskImg, SP, maskedSuperpixels);
	//maskedSuperpixels.push_back(359);

	IplImage * image0 = cvCreateImage(cvSize(width,height),8,3);
	cvZero(image0);

	for(int i = 0;i<800;i++)
	{
		std::vector<cv::Point2d> dd;
		dd.push_back(cv::Point2d(200,i));
		std::vector<cv::Point2d> ee;


		
		double xx = H.at<double>(0,0) * dd[0].x + H.at<double>(0,1) * dd[0].y + H.at<double>(0,2);
		double yy = H.at<double>(1,0) * dd[0].x + H.at<double>(1,1) * dd[0].y + H.at<double>(1,2);
		double zz = H.at<double>(2,0) * dd[0].x + H.at<double>(2,1) * dd[0].y + H.at<double>(2,2);

		double xxx = xx / zz;
		double yyy = yy / zz;

		cout<<i<<","<<xx<<","<<yy<<","<<zz<<","<<xxx<<","<<yyy<<endl;

		/*cv::perspectiveTransform(dd,ee,H);
		
		cout<<i<<","<<ee[0].x<<","<<ee[0].y<<endl;*/
	}

    // set the masked superpixels to be unknown
    for (std::vector<size_t>::iterator iter = maskedSuperpixels.begin(); iter != maskedSuperpixels.end(); iter++)
    {
        //sp_cate_label[*iter] = -1;
		sp_idx_check = *iter;
		std::cout<<sp_idx_check<<std::endl;
		std::vector<v2i> sp_check;
		sp_check = SP[sp_idx_check];
		NRDCProcessing::HomographyTransform(sp_check,H,spc);
		
		std::string checkName = "D:/hellobbb/1spc_check"+std::to_string(sp_idx_check)+".png";
		IplImage * imgggg = NRDCShow::ShowSPc(img0,sp_check,spc,checkName.c_str(), H, sp_idx_check);

		for(int i = 0;i<height;i++)
		{
			for(int j = 0;j<width;j++)
			{
				CvScalar cs;
				cs = cvGet2D(imgggg,i,j);
				if(cs.val[0] != 0 || cs.val[1] != 0 || cs.val[2] != 0)
					cvSet2D(image0,i,j,cs);
			}
		}

		cvReleaseImage(&imgggg);

    }
	cvSaveImage("D:/hellobbb/result.png",image0);
	




	//NRDCProcessing::DeleteSpGraph(&graph);

	cvReleaseImage(&img0);
	cvReleaseImage(&img1);
	cvReleaseImage(&image0);
}


void NRDCTest::TestSingleSPProperty()
{
	int width = 1450;
	int height = 779;

    std::string work_dir = "F:/LastExperiments/Case28/forward/";

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);

	NRDCProcessing::ReadSP((work_dir + "SP.txt").c_str(),SP);	
	NRDCProcessing::ReadNRDC(nrdc,(work_dir + "NRDC.txt").c_str());

	IplImage * img0 = cvLoadImage((work_dir + "a.bmp").c_str());
	IplImage * img1 = cvLoadImage((work_dir + "b.bmp").c_str());

	int sp_idx = 359;
	int sp_idx_check = 359;
	std::vector<v2d> spc;
	cv::Mat H;
	std::vector<int> sp_support;
	std::vector<double> sp_support_diff;

	// information of SP[sp_idx]
	//NRDCProcessing::ShowCorrespondence(img0,img1,nrdc,SP[sp_idx]);
	NRDCShow::ShowSP(img0,SP,sp_idx,"D:/hellobbb/spppp.png");
	NRDCShow::ShowSPc(img0,nrdc,SP,sp_idx,"D:/hellobbb/spcccccc.png");
	double coherence = NRDCProcessing::SPCoherence(nrdc,SP[sp_idx]);
	cout<<"sp_coherence:"<<coherence<<endl;
	NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);	
	//NRDCProcessing::ShowH(H);

	// check the H by other sp
	std::vector<v2i> sp_check;
	//sp_check = sp_indicate;
	sp_check = SP[sp_idx_check];
	NRDCProcessing::HomographyTransform(sp_check,H,spc);
	//NRDCProcessing::ShowCorrespondence(img0,img1,sp_check,spc,"E:/test20150709/1sp_check.png","E:/test20150709/1spc_check.png");
	NRDCShow::ShowSP(img0,sp_check,"D:/hellobbb/1sp_check.png");
	NRDCShow::ShowSPc(img0,sp_check,spc,"D:/hellobbb/1spc_check.png");
	double diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img0,img1,sp_check,spc);
	cout<<"check_diff:"<<diff<<endl;

	// check the support region of sp
	NRDCProcessing::FindSupportRegionGlobalTraversal(img0,img1,nrdc,SP,sp_idx,sp_support,sp_support_diff);
	NRDCShow::ShowSP(img0,SP,sp_support,"D:/hellobbb/2sp_support.png");
	NRDCShow::ShowSPc(img0,nrdc,SP,sp_support,"D:/hellobbb/2sp_support_homography.png");

	// check the SpGraph
	SpGraph * graph = NRDCProcessing::MakeSpGraph(SP,img0->width,img0->height);
	
	/*for(size_t i = 0;i<(*graph)[sp_idx]->neighbors.size();i++)
	{
		cout<<(*graph)[sp_idx]->neighbors[i]<<",";
	}
	cout<<endl;*/

	std::vector<v2d> N_neighbors;
	std::vector<int> N_neighbors_idx;
	NRDCProcessing::CollectN_Neighbors(graph,sp_idx,5,N_neighbors);
	for(size_t i = 0;i<N_neighbors.size();i++)
	{
		//cout<<N_neighbors[i][0]<<","<<N_neighbors[i][1]<<endl;
		N_neighbors_idx.push_back((int)(N_neighbors[i][0]));
	}
	NRDCShow::ShowSP(img0,SP,N_neighbors_idx,"D:/hellobbb/3N_neighbors.png");

	std::vector<v2d> sp_support_within_N_neighbors;
	std::vector<int> sp_support_within_N_neighbors_idx;
	cv::Mat H11;

	NRDCProcessing::FindSupportRegionWithinN_Neighbors(img0,img1,nrdc,SP,graph,sp_idx,sp_support_within_N_neighbors,H11);
	for(size_t i = 0;i<sp_support_within_N_neighbors.size();i++)
		sp_support_within_N_neighbors_idx.push_back((int)(sp_support_within_N_neighbors[i][0]));
	
	NRDCShow::ShowSP(img0,SP,sp_support_within_N_neighbors_idx,"D:/hellobbb/4sp_support.png");
	NRDCShow::ShowSPc(img0,nrdc,SP,sp_support_within_N_neighbors_idx,"D:/hellobbb/4sp_support_homography.png");


	NRDCProcessing::DeleteSpGraph(&graph);

	cvReleaseImage(&img0);
	cvReleaseImage(&img1);
}

void NRDCTest::TestComputeSupportRegion()
{
	IplImage * img_src = cvLoadImage((NRDCTest::folerpath + "/a.bmp").c_str());
	IplImage * img_dst = cvLoadImage((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src->width;
	int height = img_dst->height;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath+"/SP.txt").c_str(),SP);	
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here
	
	int sp_idx = 445;
	std::vector<v2d> sp_support;
	cv::Mat H;
	
	double co = NRDCProcessing::SPCoherence(nrdc,SP[sp_idx]);
	cout<<co<<endl;

	NRDCProcessing::FindSupportRegionWithinN_Neighbors(img_src,
		img_dst,
		nrdc,
		SP,
		graph,
		sp_idx,
		sp_support, H);

	std::vector<int> sp_support_idx;
	for(size_t i = 0;i<sp_support.size();i++)
		sp_support_idx.push_back((int)(sp_support[i][0]));

	NRDCShow::ShowSP(img_src,SP,sp_support_idx,(NRDCTest::folerpath+"/5_sp_support.png").c_str());
	NRDCShow::ShowSPc(img_src,nrdc,SP,sp_support_idx,(NRDCTest::folerpath + "/5_sp_support_correspondence.png").c_str());

	// ====================================================
	cvReleaseImage(&img_src);
	cvReleaseImage(&img_dst);
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestViewInterpolation()
{
	IplImage * img_src = cvLoadImage((NRDCTest::folerpath + "/a.bmp").c_str());
	IplImage * img_dst = cvLoadImage((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src->width;
	int height = img_dst->height;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here
	std::vector<int> seeds;
	NRDCProcessing::InitSeeds(seeds,SP,nrdc,800,600);

	NRDCProcessing::ViewInterpolation(seeds,
		img_src,
		img_dst,
		nrdc,
		SP,
		graph);

	
	// ====================================================
	cvReleaseImage(&img_src);
	cvReleaseImage(&img_dst);
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestHistogram()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here
	
	Histograms hists;
	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);

	double chi = NRDCProcessing::ChiSquareDistBtwSp(hists,363,387);	
	cout<<chi<<endl;

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestNearRegionHomoAccuracy()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here

	std::vector<int> near_sp;
	
	near_sp.push_back(428);
	near_sp.push_back(407);
	near_sp.push_back(424);
	near_sp.push_back(425);

	std::vector<std::vector<v2d>> near_sp_corre;
	
	for(size_t i = 0;i<near_sp.size();i++)
	{
		cv::Mat H;
		NRDCProcessing::ComputeHomographyForSP(nrdc,SP[near_sp[i]],H);
		std::vector<v2d> corre;
		NRDCProcessing::HomographyTransform(SP[near_sp[i]],H,corre);
		near_sp_corre.push_back(corre);
	}
	
	cv::Mat mask_img = cv::imread("E:/test20150716/mask.png");
	std::vector<cv::Point2d> near_region_coords;
	std::vector<cv::Point2d> near_region_corre; 

	for(size_t i = 0;i<near_sp.size();i++)
	{
		for(size_t j = 0;j<SP[near_sp[i]].size();j++)
		{
			v2i pos = SP[near_sp[i]][j];
			v2d posc = near_sp_corre[i][j];
			
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,mask_img,pos[0],pos[1]);

			if(r==255 && g == 0 && b == 0)
			{
				near_region_coords.push_back(cv::Point2d(pos[0],pos[1]));
				near_region_corre.push_back(cv::Point2d(posc[0],posc[1]));
			}
		}
	}

	cv::Mat new_H = cv::findHomography( cv::Mat(near_region_coords), cv::Mat(near_region_corre), CV_RANSAC );

	std::vector<int> test_sp;
	test_sp.push_back(428);
	test_sp.push_back(407);
	test_sp.push_back(424);
	test_sp.push_back(425);
	//test_sp.push_back(456);
	test_sp.push_back(449);
	test_sp.push_back(442);
	test_sp.push_back(440);
	test_sp.push_back(443);
	/*test_sp.push_back(452);
	test_sp.push_back(453);
	test_sp.push_back(454);
	test_sp.push_back(457);
	test_sp.push_back(450);
	test_sp.push_back(473);
	test_sp.push_back(466);
	test_sp.push_back(467);
	test_sp.push_back(468);
	test_sp.push_back(469);
	test_sp.push_back(474);
	test_sp.push_back(477);
	test_sp.push_back(479);
	test_sp.push_back(478);
	test_sp.push_back(480);
	test_sp.push_back(481);
	test_sp.push_back(482);
	test_sp.push_back(483);
	test_sp.push_back(484);
	test_sp.push_back(418);
	test_sp.push_back(429);
	test_sp.push_back(430);
	test_sp.push_back(414);
	test_sp.push_back(415);*/

	cv::Mat img_show;
	NRDCShow::ShowSPc(img_src,img_show,SP,test_sp,new_H);
	
	cv::imwrite("E:/test20150716/6_test_wider_region_H_accuracy.png",img_show);

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}


void NRDCTest::TestWarping()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here

	std::vector<int> data_sp;
	data_sp.push_back(428);
	data_sp.push_back(407);
	data_sp.push_back(424);
	data_sp.push_back(425);

	std::vector<int> smooth_sp;
	smooth_sp.push_back(456);
	smooth_sp.push_back(449);
	smooth_sp.push_back(442);
	smooth_sp.push_back(440);
	smooth_sp.push_back(443);
	smooth_sp.push_back(452);
	smooth_sp.push_back(453);
	smooth_sp.push_back(454);
	smooth_sp.push_back(457);
	smooth_sp.push_back(450);
	smooth_sp.push_back(473);
	smooth_sp.push_back(466);
	smooth_sp.push_back(467);
	smooth_sp.push_back(468);
	smooth_sp.push_back(469);
	smooth_sp.push_back(474);
	smooth_sp.push_back(477);
	smooth_sp.push_back(479);
	smooth_sp.push_back(478);
	smooth_sp.push_back(480);
	smooth_sp.push_back(481);
	smooth_sp.push_back(482);
	smooth_sp.push_back(483);
	smooth_sp.push_back(484);
	smooth_sp.push_back(418);
	smooth_sp.push_back(429);
	smooth_sp.push_back(430);
	smooth_sp.push_back(414);
	smooth_sp.push_back(415);

	int min_x,min_y,max_x,max_y;
	min_x = min_y = 100000;
	max_x = max_y =-100000;

	for(size_t i = 0;i<data_sp.size();i++)
	{
		for(size_t j = 0;j<SP[data_sp[i]].size();j++)
		{
			v2i pos = SP[data_sp[i]][j];
			
			if(pos[0] < min_x) min_x = pos[0];
			if(pos[0] > max_x) max_x = pos[0];
			if(pos[1] < min_y) min_y = pos[1];
			if(pos[1] > max_y) max_y = pos[1];
		}
	}

	for(size_t i = 0;i<smooth_sp.size();i++)
	{
		for(size_t j = 0;j<SP[smooth_sp[i]].size();j++)
		{
			v2i pos = SP[smooth_sp[i]][j];
			
			if(pos[0] < min_x) min_x = pos[0];
			if(pos[0] > max_x) max_x = pos[0];
			if(pos[1] < min_y) min_y = pos[1];
			if(pos[1] > max_y) max_y = pos[1];
		}
	}

	std::vector<std::vector<v2d>> data_sp_corre;
	for(size_t i = 0;i<data_sp.size();i++)
	{
		cv::Mat H;
		std::vector<v2d> corre;
		NRDCProcessing::ComputeHomographyForSP(nrdc,SP[data_sp[i]],H);
		NRDCProcessing::HomographyTransform(SP[data_sp[i]],H,corre);
		data_sp_corre.push_back(corre);		
	}

	std::vector<Triangle>	all_triangles;
	std::vector<Vertex>		all_vertices;
	std::vector<Vertex>		all_deformed_vertices;
	
	std::vector<cv::Point2f>	control_points;
	std::vector<cv::Point2f>	control_to_points;

	std::vector<FeaturePoint>	tmp_points;
	
	double lambda_data = 1;
	double lambda_smooth = 1;

	all_triangles.clear();
	all_vertices.clear();
	all_deformed_vertices.clear();
	control_points.clear();
	control_to_points.clear();
	tmp_points.clear();
	
	Geometry2D::buildTriangleMesh(min_x,max_x,min_y,max_y,all_vertices,all_triangles);
	
	/*control_points.push_back(cv::Point2f(59,468));
    control_to_points.push_back(cv::Point2f(59,468));

    control_points.push_back(cv::Point2f(117,464));
    control_to_points.push_back(cv::Point2f(117,464));

    control_points.push_back(cv::Point2f(47,504));
    control_to_points.push_back(cv::Point2f(47,504));

    control_points.push_back(cv::Point2f(168,508));
    //control_to_points.push_back(cv::Point2f(100,508));
    control_to_points.push_back(cv::Point2f(168,500));*/

	for(size_t i = 0;i<data_sp.size();i++)
	{
		for(size_t j = 0;j<SP[data_sp[i]].size();j++)
		{
			v2i pos = SP[data_sp[i]][j];
			v2d posc = data_sp_corre[i][j];

			control_points.push_back(cv::Point2f((float)(pos[0]),(float)(pos[1])));
			control_to_points.push_back(cv::Point2f((float)(posc[0]),(float)(posc[1])));
		}
	}
	
	WarpSolver ws;
	ws.perform_Warping(control_points,
		control_to_points,
		all_vertices,
		lambda_data,
		lambda_smooth,
		all_triangles,
		tmp_points,
		all_deformed_vertices);

	cv::Mat mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_vertices[tri.nVertices[0]];
		v1 = all_vertices[tri.nVertices[1]];
		v2 = all_vertices[tri.nVertices[2]];

		cv::line(mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/bbb.png",mesh_img);
	
	cv::Mat deformed_mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_deformed_vertices[tri.nVertices[0]];
		v1 = all_deformed_vertices[tri.nVertices[1]];
		v2 = all_deformed_vertices[tri.nVertices[2]];

		cv::line(deformed_mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/ccc.png",deformed_mesh_img);

	cv::Mat warp_img;
	WarpRender wr;
	//wr.drawWarpImageOffsetMesh(all_vertices,all_deformed_vertices,all_triangles,local_img,warp_img);
	wr.drawWarpImage(all_vertices,all_deformed_vertices,all_triangles,img_src,warp_img);
	
	cv::imwrite("E:/ddd.png",warp_img);

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestOurWarping()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here

	std::vector<int> data_sp;
	
	data_sp.push_back(428);
	data_sp.push_back(407);
	data_sp.push_back(424);
	data_sp.push_back(425);

	std::vector<std::vector<v2d>> data_sp_corre;
	
	for(size_t i = 0;i<data_sp.size();i++)
	{
		cv::Mat H;
		NRDCProcessing::ComputeHomographyForSP(nrdc,SP[data_sp[i]],H);
		std::vector<v2d> corre;
		NRDCProcessing::HomographyTransform(SP[data_sp[i]],H,corre);
		data_sp_corre.push_back(corre);
	}
	
	std::vector<cv::Point2d> Hp, Hpt;
	cv::Mat missing_new_H;

	Hp.push_back(cv::Point2d(200,200));
	Hp.push_back(cv::Point2d(300,200));
	Hp.push_back(cv::Point2d(200,300));
	Hp.push_back(cv::Point2d(300,300));

	Hpt.push_back(cv::Point2d(200+100,200-300));
	Hpt.push_back(cv::Point2d(300+100,200-300));
	Hpt.push_back(cv::Point2d(200+100,300-300));
	Hpt.push_back(cv::Point2d(300+100,300-300));

	missing_new_H = cv::findHomography(cv::Mat(Hp),cv::Mat(Hpt),CV_RANSAC);

	std::vector<int> missing_sp;
	missing_sp.push_back(428);
	missing_sp.push_back(407);
	missing_sp.push_back(424);
	missing_sp.push_back(425);
	missing_sp.push_back(456);
	missing_sp.push_back(449);
	missing_sp.push_back(442);
	missing_sp.push_back(440);
	missing_sp.push_back(443);
	missing_sp.push_back(452);
	missing_sp.push_back(453);
	missing_sp.push_back(454);
	missing_sp.push_back(457);
	missing_sp.push_back(450);
	missing_sp.push_back(473);
	missing_sp.push_back(466);
	missing_sp.push_back(467);
	missing_sp.push_back(468);
	missing_sp.push_back(469);
	missing_sp.push_back(474);
	missing_sp.push_back(477);
	missing_sp.push_back(479);
	missing_sp.push_back(478);
	missing_sp.push_back(480);
	missing_sp.push_back(481);
	missing_sp.push_back(482);
	missing_sp.push_back(483);
	missing_sp.push_back(484);
	missing_sp.push_back(418);
	missing_sp.push_back(429);
	missing_sp.push_back(430);
	missing_sp.push_back(414);
	missing_sp.push_back(415);

	std::vector<cv::Mat> HomoS;
	std::vector<cv::Mat> HomoT;
	HomoS.resize(graph->size());
	HomoT.resize(graph->size());
	
	for(size_t i = 0;i<missing_sp.size();i++)
	{
		HomoS[missing_sp[i]] = missing_new_H;
		HomoT[missing_sp[i]] = missing_new_H.clone();
	}

	int min_x,min_y,max_x,max_y;
	min_x = min_y = 100000;
	max_x = max_y =-100000;

	for(size_t i = 0;i<data_sp.size();i++)
	{
		for(size_t j = 0;j<SP[data_sp[i]].size();j++)
		{
			v2i pos = SP[data_sp[i]][j];
			
			if(pos[0] < min_x) min_x = pos[0];
			if(pos[0] > max_x) max_x = pos[0];
			if(pos[1] < min_y) min_y = pos[1];
			if(pos[1] > max_y) max_y = pos[1];
		}
	}

	for(size_t i = 0;i<missing_sp.size();i++)
	{
		for(size_t j = 0;j<SP[missing_sp[i]].size();j++)
		{
			v2i pos = SP[missing_sp[i]][j];
			
			if(pos[0] < min_x) min_x = pos[0];
			if(pos[0] > max_x) max_x = pos[0];
			if(pos[1] < min_y) min_y = pos[1];
			if(pos[1] > max_y) max_y = pos[1];
		}
	}
	
	std::vector<Triangle>	all_triangles;
	std::vector<Vertex>		all_vertices;
	std::vector<Vertex>		all_deformed_vertices;
	
	std::vector<Eigen::Vector2d>	control_points;
	std::vector<Eigen::Vector2d>	control_to_points;

	std::vector<FeaturePoint>	tmp_points;
	
	double lambda_data = 1;
	double lambda_smooth = 1;

	all_triangles.clear();
	all_vertices.clear();
	all_deformed_vertices.clear();
	control_points.clear();
	control_to_points.clear();
	tmp_points.clear();
	
	Geometry2D::buildTriangleMesh(min_x,max_x,min_y,max_y,all_vertices,all_triangles);

	for(size_t i = 0;i<data_sp.size();i++)
	{
		for(size_t j = 0;j<SP[data_sp[i]].size();j++)
		{
			v2i pos = SP[data_sp[i]][j];
			v2d posc = data_sp_corre[i][j];

			control_points.push_back(Eigen::Vector2d(pos[0],pos[1]));
			control_to_points.push_back(Eigen::Vector2d(posc[0],posc[1]));
		}
	}

	std::vector<std::vector<Eigen::Vector2i>> Eigen_SP;
	for(size_t i = 0; i<SP.size();i++)
	{
		std::vector<Eigen::Vector2i> tmp;
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			tmp.push_back(Eigen::Vector2i(pos[0],pos[1]));
		}
		Eigen_SP.push_back(tmp);
	}

	//PCWarpSolver pcws;
	//pcws.performWarping(img_src.cols,
	//	img_src.rows,
	//	all_vertices,
	//	all_deformed_vertices,
	//	all_triangles,
	//	control_points,
	//	control_to_points,
	//	Eigen_SP,
	//	HomoS,
	//	missing_sp,
	//	HomoT,
	//	1,
	//	1e1,
	//	1e3,
	//	1e1);

	cv::Mat mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_vertices[tri.nVertices[0]];
		v1 = all_vertices[tri.nVertices[1]];
		v2 = all_vertices[tri.nVertices[2]];

		cv::line(mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestOurWarping.src.mesh.png",mesh_img);
	
	cv::Mat deformed_mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_deformed_vertices[tri.nVertices[0]];
		v1 = all_deformed_vertices[tri.nVertices[1]];
		v2 = all_deformed_vertices[tri.nVertices[2]];

		cv::line(deformed_mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestOurWarping.dst.mesh.png",deformed_mesh_img);

	cv::Mat warp_img;
	WarpRender wr;
	//wr.drawWarpImageOffsetMesh(all_vertices,all_deformed_vertices,all_triangles,local_img,warp_img);
	wr.drawWarpImage(all_vertices,all_deformed_vertices,all_triangles,img_src,warp_img);
	
	cv::imwrite("E:/test20150716/TestOurWarping.rendering.png",warp_img);

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestDijkstra()
{
	SpGraph * graph = new SpGraph();
	graph->resize(6);

	(*graph)[0] = new SpNode();
	(*graph)[0]->sp_id = 0;
	(*graph)[0]->neighbors.push_back(1);
	(*graph)[0]->neighbors.push_back(2);
	(*graph)[0]->neighbors.push_back(3);
	
	(*graph)[1] = new SpNode();
	(*graph)[1]->sp_id = 1;
	(*graph)[1]->neighbors.push_back(0);
	(*graph)[1]->neighbors.push_back(4);

	(*graph)[2] = new SpNode();
	(*graph)[2]->sp_id = 2;
	(*graph)[2]->neighbors.push_back(0);
	(*graph)[2]->neighbors.push_back(5);

	(*graph)[3] = new SpNode();
	(*graph)[3]->sp_id = 3;
	(*graph)[3]->neighbors.push_back(0);
	//(*graph)[3]->neighbors.push_back(5);

	(*graph)[4] = new SpNode();
	(*graph)[4]->sp_id = 4;
	(*graph)[4]->neighbors.push_back(1);
	(*graph)[4]->neighbors.push_back(5);

	(*graph)[5] = new SpNode();
	(*graph)[5]->sp_id = 5;
	//(*graph)[5]->neighbors.push_back(3);
	(*graph)[5]->neighbors.push_back(4);
	(*graph)[5]->neighbors.push_back(2);

	NRDCProcessing::ClearGraphTmp(graph);

	SpChiDiff sp_chi_diff;
	sp_chi_diff.set(6,6);
	sp_chi_diff.clear(0);

	sp_chi_diff.at(0,1) = 2;
	sp_chi_diff.at(0,2) = 3;
	sp_chi_diff.at(0,3) = 1;
	
	sp_chi_diff.at(1,0) = 2;
	sp_chi_diff.at(1,4) = 3;

	sp_chi_diff.at(2,0) = 3;
	sp_chi_diff.at(2,5) = 2;

	sp_chi_diff.at(3,0) = 1;
	//sp_chi_diff.at(3,5) = 4;

	sp_chi_diff.at(4,1) = 3;
	sp_chi_diff.at(4,5) = 9;

	sp_chi_diff.at(5,2) = 2;
	//sp_chi_diff.at(5,3) = 4;
	sp_chi_diff.at(5,4) = 9;

	/*SpGraph * graph = new SpGraph();
	graph->resize(6);

	(*graph)[0] = new SpNode();
	(*graph)[0]->sp_id = 0;
	(*graph)[0]->neighbors.push_back(1);
	(*graph)[0]->neighbors.push_back(2);
	(*graph)[0]->neighbors.push_back(5);
	
	(*graph)[1] = new SpNode();
	(*graph)[1]->sp_id = 1;
	(*graph)[1]->neighbors.push_back(0);
	(*graph)[1]->neighbors.push_back(2);
	(*graph)[1]->neighbors.push_back(3);

	(*graph)[2] = new SpNode();
	(*graph)[2]->sp_id = 2;
	(*graph)[2]->neighbors.push_back(0);
	(*graph)[2]->neighbors.push_back(1);
	(*graph)[2]->neighbors.push_back(3);
	(*graph)[2]->neighbors.push_back(5);

	(*graph)[3] = new SpNode();
	(*graph)[3]->sp_id = 3;
	(*graph)[3]->neighbors.push_back(1);
	(*graph)[3]->neighbors.push_back(2);
	(*graph)[3]->neighbors.push_back(4);

	(*graph)[4] = new SpNode();
	(*graph)[4]->sp_id = 4;
	(*graph)[4]->neighbors.push_back(3);
	(*graph)[4]->neighbors.push_back(5);

	(*graph)[5] = new SpNode();
	(*graph)[5]->sp_id = 5;
	(*graph)[5]->neighbors.push_back(0);
	(*graph)[5]->neighbors.push_back(2);
	(*graph)[5]->neighbors.push_back(4);

	NRDCProcessing::ClearGraphTmp(graph);

	SpChiDiff sp_chi_diff;
	sp_chi_diff.set(6,6);
	sp_chi_diff.clear(0);

	sp_chi_diff.at(0,1) = 7;
	sp_chi_diff.at(0,2) = 9;
	sp_chi_diff.at(0,5) = 14;
	
	sp_chi_diff.at(1,0) = 7;
	sp_chi_diff.at(1,2) = 10;
	sp_chi_diff.at(1,3) = 15;

	sp_chi_diff.at(2,0) = 9;
	sp_chi_diff.at(2,1) = 10;
	sp_chi_diff.at(2,3) = 11;
	sp_chi_diff.at(2,5) = 2;

	sp_chi_diff.at(3,1) = 15;
	sp_chi_diff.at(3,2) = 11;
	sp_chi_diff.at(3,4) = 6;

	sp_chi_diff.at(4,3) = 6;
	sp_chi_diff.at(4,5) = 9;

	sp_chi_diff.at(5,0) = 14;
	sp_chi_diff.at(5,2) = 2;
	sp_chi_diff.at(5,4) = 9;*/
	// =======================================================================
	// Processs begins at this
	
	SpNode * path_tree;
	NRDCProcessing::DijkstraShortestPath(graph,sp_chi_diff,0,path_tree);

	for(size_t i = 0;i<graph->size();i++)
	{
		SpNode * node = (*graph)[i];

		while(node != 0)
		{
			cout<<node->sp_id<<" ("<<node->tmp0<<") ";
			node = (node->parent_nodes.size() > 0) ? node->parent_nodes[0] : 0;
		}
		cout<<endl;
	}

	// ========================================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestDivideSpCategory()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here
	
	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);

	cv::Mat img_show0(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	cv::Mat img_show1(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	cv::Mat img_show2(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));

	for(size_t i = 0;i<sp_cate_label.size();i++)
	{
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);

			if(sp_cate_label[i] >= 0)
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show0,pos[0],pos[1]);
			if(sp_cate_label[i] == 0)
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show1,pos[0],pos[1]);
			if(sp_cate_label[i] ==-1) 
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show2,pos[0],pos[1]);
				
		}
	}

	
	cv::imwrite("E:/test20150716/7.coherent_sp.png",img_show0);
	cv::imwrite("E:/test20150716/7.boundary_sp.png",img_show1);
	cv::imwrite("E:/test20150716/7.missing_sp.png",img_show2);
	
	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestNearestBoundarySp()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	// ====================================================
	// processing begins at here
	
	int missing_sp_idx = 382;
	int nearest_boundary_idx;

	Histograms hists;
	SpChiDiff sp_chi_diff;
	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);

	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);
	
	std::vector<int> sp_mask;
	sp_mask.resize(graph->size(),0);
	sp_mask[349] = 1;
	sp_mask[350] = 1;
	sp_mask[339] = 1;
	sp_mask[366] = 1;
	sp_mask[355] = 1;
	sp_mask[363] = 1;
	sp_mask[367] = 1;
	sp_mask[382] = 1;
	sp_mask[402] = 1;
	sp_mask[381] = 1;
	sp_mask[387] = 1;
	sp_mask[380] = 1;
	sp_mask[388] = 1;
	sp_mask[411] = 1;
	sp_mask[408] = 1;
	sp_mask[412] = 1;
	sp_mask[416] = 1;
	sp_mask[405] = 1;
	sp_mask[406] = 1;
	sp_mask[441] = 1;
	sp_mask[445] = 1;
	sp_mask[431] = 1;
	sp_mask[451] = 1;
	
	NRDCProcessing::ComputeNearestBoundarySpForMissingSp(graph,sp_chi_diff,hists,sp_cate_label,sp_mask,
		missing_sp_idx,nearest_boundary_idx);

	cv::Mat img_show(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<SP[nearest_boundary_idx].size();i++)
	{
		v2i pos = SP[nearest_boundary_idx][i];
		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
		LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,pos[0],pos[1]);
	}

	cv::imwrite("E:/test20150716/8.nearest_boundary_sp.png",img_show);

	// test homo inferrence seeds
	std::vector<int> seeds;
	NRDCProcessing::ComputeSeedsForHomographyInference(graph,sp_chi_diff,sp_cate_label,sp_mask,nearest_boundary_idx,seeds);

	img_show.setTo(cv::Scalar(0,0,0));	

	for(size_t j = 0;j<seeds.size();j++)
	{
		for(size_t i = 0;i<SP[seeds[j]].size();i++)
		{
			v2i pos = SP[seeds[j]][i];
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
			LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,pos[0],pos[1]);
		}
	}
	
	cv::imwrite("E:/test20150716/8.nearest_boundary_seeds.png",img_show);

	/*std::vector<v2i> hist_matching_samples;
	NRDCProcessing::HistogramMatching(img_src,SP,seeds,missing_sp_idx,hist_matching_samples);

	img_show.setTo(cv::Scalar(0,0,0));
	for(size_t i = 0;i<hist_matching_samples.size();i++)
	{
		v2i pos = hist_matching_samples[i];
		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
		LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,pos[0],pos[1]);
	}

	cv::imwrite("E:/test20150716/8.samples_for_H_estimate.png",img_show);*/
	
	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestAllHomograpies()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	Histograms hists;
	SpChiDiff sp_chi_diff;

	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);

	// ====================================================
	// processing begins at here

	std::vector<cv::Mat> homos;
	homos.resize(graph->size());
	
	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);

	std::vector<int> sp_mask;
	sp_mask.resize(graph->size(),0);
	sp_mask[349] = 1;
	sp_mask[350] = 1;
	sp_mask[339] = 1;
	sp_mask[366] = 1;
	sp_mask[355] = 1;
	sp_mask[363] = 1;
	sp_mask[367] = 1;
	sp_mask[382] = 1;
	sp_mask[402] = 1;
	sp_mask[381] = 1;
	sp_mask[387] = 1;
	sp_mask[380] = 1;
	sp_mask[388] = 1;
	sp_mask[411] = 1;
	sp_mask[408] = 1;
	sp_mask[412] = 1;
	sp_mask[416] = 1;
	sp_mask[405] = 1;
	sp_mask[406] = 1;
	sp_mask[441] = 1;
	sp_mask[445] = 1;
	sp_mask[431] = 1;
	sp_mask[451] = 1;
	
	for(size_t i = 0;i<sp_cate_label.size();i++)
	{
		cout<<i<<endl;
		if(sp_cate_label[i] >= 0) 
		{
			cv::Mat H;
			NRDCProcessing::ComputeHomographyForSP(nrdc,SP[i],H);
			homos[i] = H;
		}
		else
		{
			int nearest_boundary_idx;
			std::vector<int> H_seeds;
			cv::Mat H;			

			NRDCProcessing::ComputeNearestBoundarySpForMissingSp(graph,
				sp_chi_diff,
				hists,
				sp_cate_label,
				sp_mask,
				(int)i,
				nearest_boundary_idx);
		
			NRDCProcessing::ComputeSeedsForHomographyInference(graph,sp_chi_diff,sp_cate_label,
				sp_mask,
				nearest_boundary_idx,H_seeds);
			
			
			NRDCProcessing::ComputeHomographyForSP(nrdc,SP,H_seeds,H);

			homos[i] = H;
		}
	}

	cv::Mat img_show_known,img_show_missing, img_show_all;
	std::vector<int> all_seeds_known, all_seeds_missing, all_seeds;
	std::vector<cv::Mat> mat_known, mat_missing, mat_all;
	
	for(size_t i = 0;i<graph->size();i++)
	{
		if(sp_mask[i] == 0)		
		{
			if(sp_cate_label[i] >= 0)
			{
				all_seeds_known.push_back((int)i);
				mat_known.push_back(homos[i]);
			}
			else
			{
				all_seeds_missing.push_back((int)i);
				mat_missing.push_back(homos[i]);
			}
			all_seeds.push_back((int)i);
			mat_all.push_back(homos[i]);
		}
	}
	
	for(size_t i = 0;i<graph->size();i++)
	{
		if(sp_mask[i] == 1)		
		{
			if(sp_cate_label[i] >= 0)
			{
				all_seeds_known.push_back((int)i);
				mat_known.push_back(homos[i]);
			}
			else
			{
				all_seeds_missing.push_back((int)i);
				mat_missing.push_back(homos[i]);
			}
			all_seeds.push_back((int)i);
			mat_all.push_back(homos[i]);
		}
	}

	//NRDCShow::ShowSPc(img_src,img_show_known,SP,all_seeds_known,mat_known);
	//NRDCShow::ShowSPc(img_src,img_show_missing,SP,all_seeds_missing,mat_missing);
	NRDCShow::ShowSPc(img_src,img_show_all,SP,all_seeds,mat_all);
	
	//cv::imwrite("E:/test20150716/9.homographies_known.png",img_show_known);
	//cv::imwrite("E:/test20150716/9.homographies_missing.png",img_show_missing);
	cv::imwrite("E:/test20150716/9.homographies_all.png",img_show_all);
	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestCarWarping()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	Histograms hists;
	SpChiDiff sp_chi_diff;

	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);

	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);



	// ====================================================
	// processing begins at here

	cout<<"TestCarWarping ... Processing begins at here ... "<<std::flush;
	std::vector<int> sp_mask;
	sp_mask.resize(graph->size(),0);
	sp_mask[349] = 1;
	sp_mask[350] = 1;
	sp_mask[339] = 1;
	sp_mask[366] = 1;
	sp_mask[355] = 1;
	sp_mask[363] = 1;
	sp_mask[367] = 1;
	sp_mask[382] = 1;
	sp_mask[402] = 1;
	sp_mask[381] = 1;
	sp_mask[387] = 1;
	sp_mask[380] = 1;
	sp_mask[388] = 1;
	sp_mask[411] = 1;
	sp_mask[408] = 1;
	sp_mask[412] = 1;
	sp_mask[416] = 1;
	sp_mask[405] = 1;
	sp_mask[406] = 1;
	sp_mask[441] = 1;
	sp_mask[445] = 1;
	sp_mask[431] = 1;
	sp_mask[451] = 1;
	
	std::vector<cv::Mat> homoS_all;
	std::vector<cv::Mat> homoT;
	homoS_all.resize(graph->size());
	homoT.resize(graph->size());
	
	std::vector<int> sp_idx_of_missing;
	std::vector<int> sp_idx_of_known;

	cout<<"compute homography and collect information ..."<<std::flush;
	for(size_t i = 0;i<sp_mask.size();i++)
	{
		if(sp_mask[i] == 0)
			continue;

		cout<<i<<"..."<<std::flush;

		int sp_idx = (int)i;	

		if(sp_cate_label[sp_idx] >= 0) // coherent
		{
			cv::Mat H;
			NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);
			homoS_all[sp_idx] = H;
			
			sp_idx_of_known.push_back(sp_idx);
		}
		else
		{
			int nearest_boundary_idx;
			std::vector<int> H_seeds;
			cv::Mat H;			

			NRDCProcessing::ComputeNearestBoundarySpForMissingSp(graph,
				sp_chi_diff,
				hists,
				sp_cate_label,
				sp_mask,
				sp_idx,
				nearest_boundary_idx);
		
			NRDCProcessing::ComputeSeedsForHomographyInference(graph,sp_chi_diff,sp_cate_label,
				sp_mask,
				nearest_boundary_idx,H_seeds);
			
			NRDCProcessing::ComputeHomographyForSP(nrdc,SP,H_seeds,H);

			homoS_all[sp_idx] = H;
			homoT[sp_idx] = H.clone();

			sp_idx_of_missing.push_back(sp_idx);
		}
	}

	int min_x,min_y,max_x,max_y;
	min_x = min_y = 100000;
	max_x = max_y =-100000;

	for(size_t i = 0;i<sp_mask.size();i++)
	{
		if(sp_mask[i] == 0)
			continue;

		int sp_idx = (int)i;
		
		for(size_t j = 0;j<SP[sp_idx].size();j++)
		{
			v2i pos = SP[sp_idx][j];
			
			if(pos[0] < min_x) min_x = pos[0];
			if(pos[0] > max_x) max_x = pos[0];
			if(pos[1] < min_y) min_y = pos[1];
			if(pos[1] > max_y) max_y = pos[1];
		}
	}

	std::vector<Triangle>	all_triangles;
	std::vector<Vertex>		all_vertices;
	std::vector<Vertex>		all_deformed_vertices;
	
	std::vector<Eigen::Vector2d>	control_points;
	std::vector<Eigen::Vector2d>	control_to_points;

	std::vector<FeaturePoint>	tmp_points;
	
	all_triangles.clear();
	all_vertices.clear();
	all_deformed_vertices.clear();
	control_points.clear();
	control_to_points.clear();
	tmp_points.clear();
	
	cout<<"Build triangle mesh ..."<<endl;
	Geometry2D::buildTriangleMesh(min_x,max_x,min_y,max_y,all_vertices,all_triangles);

	// data term by control points
	for(size_t i = 0;i<sp_idx_of_known.size();i++)
	{
		int sp_idx = sp_idx_of_known[i];
		
		std::vector<v2d> corre;
		NRDCProcessing::HomographyTransform(SP[sp_idx],homoS_all[sp_idx],corre);

		for(size_t j = 0;j<corre.size();j+=50)
		{
			v2i pos  = SP[sp_idx][j];
			v2d posc = corre[j];

			control_points.push_back(Eigen::Vector2d(pos[0],pos[1]));
			control_to_points.push_back(Eigen::Vector2d(posc[0],posc[1]));
		}
	}

	std::vector<std::vector<Eigen::Vector2i>> Eigen_SP;
	for(size_t i = 0; i<SP.size();i++)
	{
		std::vector<Eigen::Vector2i> tmp;
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			tmp.push_back(Eigen::Vector2i(pos[0],pos[1]));
		}
		Eigen_SP.push_back(tmp);
	}

	cout<<"Warping ..."<<std::flush;
	//PCWarpSolver pcws;
	//pcws.performWarping(img_src.cols,
	//	img_src.rows,
	//	all_vertices,
	//	all_deformed_vertices,
	//	all_triangles,
	//	control_points,
	//	control_to_points,
	//	Eigen_SP,
	//	homoS_all,
	//	sp_idx_of_missing,
	//	homoT,
	//	1,
	//	1,
	//	1e5,
	//	1e5);

	cout<<"Image storaging ..."<<std::flush;
	cv::Mat mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_vertices[tri.nVertices[0]];
		v1 = all_vertices[tri.nVertices[1]];
		v2 = all_vertices[tri.nVertices[2]];

		cv::line(mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestCarWarping.src.mesh.png",mesh_img);
	
	cv::Mat deformed_mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_deformed_vertices[tri.nVertices[0]];
		v1 = all_deformed_vertices[tri.nVertices[1]];
		v2 = all_deformed_vertices[tri.nVertices[2]];

		cv::line(deformed_mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestCarWarping.dst.mesh.png",deformed_mesh_img);


	cv::Mat img_local(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	for(size_t i = 0;i<sp_mask.size();i++)
	{
		if(sp_mask[i] == 0) 
			continue;
		
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
			LibIV::CppHelpers::Global::setImgData(r,g,b,img_local,pos[0],pos[1]);
		}
	}

	cv::Mat warp_img;
	WarpRender wr;
	wr.drawWarpImage(all_vertices,all_deformed_vertices,all_triangles,img_local,warp_img);
	
	cv::imwrite("E:/test20150716/TestCarWarping.rendering.png",warp_img);

	cout<<"End!"<<endl;
	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestLocalWarping()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	Histograms hists;
	SpChiDiff sp_chi_diff;

	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);

	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);

	// ====================================================
	// processing begins at here

	cout<<"TestLocalWarping ... Processing begins at here ... "<<std::flush;
	std::vector<int> sp_mask;
	sp_mask.resize(graph->size(),0);
	sp_mask[349] = 1;
	sp_mask[350] = 1;
	sp_mask[339] = 1;
	sp_mask[366] = 1;
	sp_mask[355] = 1;
	sp_mask[363] = 1;
	sp_mask[367] = 1;
	sp_mask[382] = 1;
	sp_mask[402] = 1;
	sp_mask[381] = 1;
	sp_mask[387] = 1;
	sp_mask[380] = 1;
	sp_mask[388] = 1;
	sp_mask[411] = 1;
	sp_mask[408] = 1;
	sp_mask[412] = 1;
	sp_mask[416] = 1;
	sp_mask[405] = 1;
	sp_mask[406] = 1;
	sp_mask[441] = 1;
	sp_mask[445] = 1;
	sp_mask[431] = 1;
	sp_mask[451] = 1;
	
	std::vector<Triangle> all_triangles;
	std::vector<Vertex>	  all_vertices;
	std::vector<Vertex>	  all_deformed_vertices;
	cv::Mat img_local;

	NRDCProcessing::LocalWarping(img_src,
		SP,
		nrdc,
		graph,
		hists,
		sp_chi_diff,
		sp_mask,
		sp_cate_label,
		img_local,
		all_triangles,
		all_vertices,
		all_deformed_vertices);
		
	cout<<"Image storaging ..."<<std::flush;
	cv::Mat mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_vertices[tri.nVertices[0]];
		v1 = all_vertices[tri.nVertices[1]];
		v2 = all_vertices[tri.nVertices[2]];

		cv::line(mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestLocalWarping.src.mesh.png",mesh_img);
	
	cv::Mat deformed_mesh_img(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
	
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		Triangle tri = all_triangles[i];
		
		Vertex v0,v1,v2;
		v0 = all_deformed_vertices[tri.nVertices[0]];
		v1 = all_deformed_vertices[tri.nVertices[1]];
		v2 = all_deformed_vertices[tri.nVertices[2]];

		cv::line(deformed_mesh_img,
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v1.vPosition.x()),(int)(v1.vPosition.y())),
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Scalar(255,0,0));
		cv::line(deformed_mesh_img,
			cv::Point((int)(v2.vPosition.x()),(int)(v2.vPosition.y())),
			cv::Point((int)(v0.vPosition.x()),(int)(v0.vPosition.y())),
			cv::Scalar(255,0,0));
	}

	cv::imwrite("E:/test20150716/TestLocalWarping.dst.mesh.png",deformed_mesh_img);

	cv::Mat warp_img;
	WarpRender wr;
	wr.drawWarpImage(all_vertices,all_deformed_vertices,all_triangles,img_local,warp_img);
	
	cv::imwrite("E:/test20150716/TestLocalWarping.rendering.png",warp_img);

	cout<<"End!"<<endl;

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestAllWarping()
{
	cv::Mat img_src = cv::imread((NRDCTest::folerpath + "/a.bmp").c_str());
	cv::Mat img_dst = cv::imread((NRDCTest::folerpath + "/b.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((NRDCTest::folerpath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(NRDCTest::folerpath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	Histograms hists;
	SpChiDiff sp_chi_diff;

	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);

	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);

	// ====================================================
	// processing begins at here

	cout<<"TestLocalWarping ... Processing begins at here ... "<<std::flush;
	std::vector<int> sp_mask_car, sp_mask_ground, sp_mask_sky, sp_mask_other;
	sp_mask_car.resize(graph->size(),0);
	sp_mask_ground.resize(graph->size(),0);
	sp_mask_sky.resize(graph->size(),0);
	sp_mask_other.resize(graph->size(),0);

	sp_mask_car[349] = 1;
	sp_mask_car[350] = 1;
	sp_mask_car[339] = 1;
	sp_mask_car[366] = 1;
	sp_mask_car[355] = 1;
	sp_mask_car[363] = 1;
	sp_mask_car[367] = 1;
	sp_mask_car[382] = 1;
	sp_mask_car[402] = 1;
	sp_mask_car[381] = 1;
	sp_mask_car[387] = 1;
	sp_mask_car[380] = 1;
	sp_mask_car[388] = 1;
	sp_mask_car[411] = 1;
	sp_mask_car[408] = 1;
	sp_mask_car[412] = 1;
	sp_mask_car[416] = 1;
	sp_mask_car[405] = 1;
	sp_mask_car[406] = 1;
	sp_mask_car[441] = 1;
	sp_mask_car[445] = 1;
	sp_mask_car[431] = 1;
	sp_mask_car[451] = 1;

	sp_mask_ground[418] = 1;
	sp_mask_ground[428] = 1;
	sp_mask_ground[407] = 1;
	sp_mask_ground[424] = 1;
	sp_mask_ground[425] = 1;
	sp_mask_ground[429] = 1;
	sp_mask_ground[430] = 1;
	sp_mask_ground[414] = 1;
	sp_mask_ground[415] = 1;
	sp_mask_ground[456] = 1;
	sp_mask_ground[449] = 1;
	sp_mask_ground[442] = 1;
	sp_mask_ground[440] = 1;
	sp_mask_ground[443] = 1;
	sp_mask_ground[452] = 1;
	sp_mask_ground[453] = 1;
	sp_mask_ground[454] = 1;
	sp_mask_ground[457] = 1;
	sp_mask_ground[450] = 1;
	sp_mask_ground[473] = 1;
	sp_mask_ground[466] = 1;
	sp_mask_ground[467] = 1;
	sp_mask_ground[468] = 1;
	sp_mask_ground[469] = 1;
	sp_mask_ground[474] = 1;
	sp_mask_ground[477] = 1;
	sp_mask_ground[479] = 1;
	sp_mask_ground[478] = 1;
	sp_mask_ground[480] = 1;
	sp_mask_ground[481] = 1;
	sp_mask_ground[482] = 1;
	sp_mask_ground[483] = 1;
	sp_mask_ground[484] = 1;
	sp_mask_ground[444] = 1;
	sp_mask_ground[446] = 1;
	sp_mask_ground[475] = 1;
	sp_mask_ground[476] = 1;
	sp_mask_ground[470] = 1;
	sp_mask_ground[461] = 1;
	sp_mask_ground[462] = 1;
	sp_mask_ground[471] = 1;
	sp_mask_ground[463] = 1;
	sp_mask_ground[459] = 1;
	sp_mask_ground[464] = 1;
	sp_mask_ground[458] = 1;
	sp_mask_ground[460] = 1;
	sp_mask_ground[465] = 1;
	sp_mask_ground[472] = 1;
	sp_mask_ground[433] = 1;
	sp_mask_ground[434] = 1;
	sp_mask_ground[435] = 1;
	sp_mask_ground[436] = 1;
	sp_mask_ground[438] = 1;
	sp_mask_ground[447] = 1;
	sp_mask_ground[437] = 1;
	sp_mask_ground[448] = 1;
	sp_mask_ground[455] = 1;
	sp_mask_ground[439] = 1;
	sp_mask_ground[432] = 1;
	sp_mask_ground[409] = 1;
	sp_mask_ground[417] = 1;
	sp_mask_ground[413] = 1;
	sp_mask_ground[426] = 1;
	sp_mask_ground[427] = 1;
	sp_mask_ground[423] = 1;
	sp_mask_ground[410] = 1;
	sp_mask_ground[420] = 1;
	sp_mask_ground[421] = 1;
	sp_mask_ground[422] = 1;
	sp_mask_ground[419] = 1;

	for(int i = 1;i<61;i++)
		sp_mask_sky[i] = 1;
	sp_mask_sky[63] = 1;
	sp_mask_sky[66] = 1;
	sp_mask_sky[69] = 1;
	sp_mask_sky[73] = 1;
	sp_mask_sky[64] = 1;
	sp_mask_sky[83] = 1;
	sp_mask_sky[75] = 1;

	for(size_t i = 0;i<graph->size();i++)
	{
		if(sp_mask_ground[i] == 1 && sp_mask_car[i] == 1 && sp_mask_sky[i] == 1)
			cout<<" ======================== error"<<endl;
		if(sp_mask_ground[i] == 1 || sp_mask_car[i] == 1 || sp_mask_sky[i] == 1)
			sp_mask_other[i] = 0;
		else
			sp_mask_other[i] = 1;
	}


	LibIV::Memory::Array::FastArray2D<v3d> src_dst_corre;
	src_dst_corre.set(img_src.cols,img_src.rows);
	src_dst_corre.fill(_v3d_(0,0,0));

	NRDCProcessing::LocalWarping2(
		img_src,
		SP,
		nrdc,
		graph,
		hists,
		sp_chi_diff,
		sp_mask_other,
		sp_cate_label,
		src_dst_corre,
		1);

	
	NRDCProcessing::LocalWarping2(
		img_src,
		SP,
		nrdc,
		graph,
		hists,
		sp_chi_diff,
		sp_mask_ground,
		sp_cate_label,
		src_dst_corre,
		2);
	

	NRDCProcessing::LocalWarping2(
		img_src,
		SP,
		nrdc,
		graph,
		hists,
		sp_chi_diff,
		sp_mask_car,
		sp_cate_label,
		src_dst_corre,
		3);
	
	NRDCProcessing::ViewInterpolation("E:/test20150716",img_src,src_dst_corre,3);

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
}

void NRDCTest::TestBidirectional()
{
	std::vector<int> segmentation_label_forward;
	std::vector<int> segmentation_label_backward;


	// forward segmentation
	int car_idx_forward[23] = 
						 {349,350,339,366,355,363,367,382,402,381,
					      387,380,388,411,408,412,416,405,406,441,
					      445,431,451};

	int ground_idx_forward[70] = 
					   	 {418,428,407,424,425,429,430,414,415,456,
		                  449,442,440,443,452,453,454,457,450,473,
						  466,467,468,469,474,477,479,478,480,481,
						  482,483,484,444,446,475,476,470,461,462,
						  471,463,459,464,458,460,465,472,433,434,
						  435,436,438,447,437,448,455,439,432,409,
						  417,413,426,427,423,410,420,421,422,419};

	int sky_idx_forward[67] = 
						 {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
		                  11,12,13,14,15,16,17,18,19,20,
						  21,22,23,24,25,26,27,28,29,30,
						  31,32,33,34,35,36,37,38,39,40,
						  41,42,43,44,45,46,47,48,49,50,
						  51,52,53,54,55,56,57,58,59,60,
						  63,66,69,73,64,83,75};
	
	segmentation_label_forward.resize(485,1);
		
	for(int i = 0;i<67;i++)
		segmentation_label_forward[sky_idx_forward[i]] = 0;

	for(int i = 0;i<70;i++)
		segmentation_label_forward[ground_idx_forward[i]] = 2;

	for(int i = 0;i<23;i++)
		segmentation_label_forward[car_idx_forward[i]] = 3;

	// backward segmentation 
	int car_idx_backward[30] =  
					   {347,349,332,325,355,352,354,359,377,380,
						379,386,406,404,400,399,391,411,415,416,
						431,429,408,436,451,447,448,435,412,356};
	int ground_idx_backward[65] =
					   {428,421,407,430,418,445,439,437,440,446,
						478,472,470,473,481,479,484,485,471,482,
						483,466,441,458,452,432,433,434,438,442,
						443,453,449,454,444,450,456,457,455,474,
						459,460,462,463,467,464,468,461,465,475,
						476,480,469,477,413,427,419,420,423,417,
						424,425,426,422,414};
	int sky_idx_backward[48] =  
					   {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
		                11,12,13,14,15,16,17,18,19,20,
						21,22,23,50,31,28,30,32,33,34,
						35,36,38,39,40,41,42,45,46,48,
						49,51,52,53,43,54,29,55};

	segmentation_label_backward.resize(486,1);
		
	for(int i = 0;i<48;i++)
		segmentation_label_backward[sky_idx_backward[i]] = 0;

	for(int i = 0;i<65;i++)
		segmentation_label_backward[ground_idx_backward[i]] = 2;

	for(int i = 0;i<30;i++)
		segmentation_label_backward[car_idx_backward[i]] = 3;

	cv::Mat img_src_forward, img_src_backward;
	LibIV::Memory::Array::FastArray2D<v3d> src_dst_corre_forward, src_dst_corre_backward;
	
	NRDCProcessing::ComputeSrcDstCompleteCorre("D:/test20150716",segmentation_label_forward,img_src_forward,src_dst_corre_forward);
	NRDCProcessing::ComputeSrcDstCompleteCorre("D:/test20150727",segmentation_label_backward,img_src_backward,src_dst_corre_backward);
	
	// save the correspondence in file for time saving
	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	ofstream outfile_forward_seg_label("D:/test20150727/correspondence/segmentation_label_forward.txt");
	for(size_t i = 0;i<segmentation_label_forward.size();i++)
		outfile_forward_seg_label<<segmentation_label_forward[i]<<'\n';
	outfile_forward_seg_label.close();

	ofstream outfile_forward_corre("D:/test20150727/correspondence/forward_corre.txt",ios::binary);
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

	ofstream outfile_backward_seg_label("D:/test20150727/correspondence/segmentation_label_backward.txt");
	for(size_t i = 0;i<segmentation_label_backward.size();i++)
		outfile_backward_seg_label<<segmentation_label_backward[i]<<'\n';
	outfile_backward_seg_label.close();

	ofstream outfile_backward_corre("D:/test20150727/correspondence/backward_corre.txt",ios::binary);
	for(int i = 0;i<img_src_forward.rows;i++)
	{
		for(int j = 0;j<img_src_forward.cols;j++)
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
	

	/*NRDCProcessing::ViewInterpolation("E:/test20150727",3,img_src_forward,img_src_backward,
		src_dst_corre_forward,
		src_dst_corre_backward);*/
	//NRDCProcessing::ViewInterpolation("E:/test20150727",img_src_forward,src_dst_corre_forward,3);
	//NRDCProcessing::ViewInterpolation("E:/test20150727",img_src_backward,src_dst_corre_backward,3);

}

void NRDCTest::TestReadCorre()
{
	cv::Mat img_src_forward  = cv::imread("D:/test20150716/a.bmp");

    std::cout << img_src_forward.cols << std::endl;


	cv::Mat img_src_backward = cv::imread("D:/test20150727/a.bmp");
	
	std::vector<int> seg_label_forward, seg_label_backward;
	
	string line;
	ifstream infile("D:/test20150727/correspondence/segmentation_label_forward.txt");
	if(infile)
	{
		while(getline(infile,line) && line != "")
		{
			int label;
			istringstream(line)>>label;
			seg_label_forward.push_back(label);
		}
		infile.close();
	}
	else
	{
		return;
	}

	infile.open("D:/test20150727/correspondence/segmentation_label_backward.txt");
	if(infile)
	{
		while(getline(infile,line) && line != "")
		{
			int label;
			istringstream(line)>>label;
			seg_label_backward.push_back(label);
		}
		infile.close();
	}
	else
	{
		return;
	}

	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	LibIV::Memory::Array::FastArray2D<v3d> corre_forward, corre_backward;
	corre_forward.set(img_src_forward.cols,img_src_forward.rows);
	corre_forward.fill(_v3d_(0,0,0));

	corre_backward.set(img_src_backward.cols,img_src_backward.rows);
	corre_backward.fill(_v3d_(0,0,0));

	long long int ss;
	char * buf;
	char * p;
	infile.open("D:/test20150727/correspondence/forward_corre.txt",ios::binary);
	if(infile)
	{
		infile.seekg(0,ios::end);
		ss = infile.tellg();
		buf = new char[(unsigned int)ss];
		infile.seekg(0,ios::beg);
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

	infile.open("D:/test20150727/correspondence/backward_corre.txt",ios::binary);
	if(infile)
	{
		infile.seekg(0,ios::end);
		ss = infile.tellg();
		buf = new char[(unsigned int)ss];
		infile.seekg(0,ios::beg);
		infile.read(buf,ss);
		p = buf;
		
		for(int i = 0;i<img_src_forward.rows;i++)
		{
			for(int j = 0;j<img_src_forward.cols;j++)
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

	NRDCProcessing::ViewInterpolation("D:/test20150727",3,img_src_forward,img_src_backward,
		corre_forward,
		corre_backward);
}