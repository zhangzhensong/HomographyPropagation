#include <fstream>
#include <set>
#include <queue>
#include <omp.h>

#include "nrdc_processing.h"
#include "nrdc_test.h"
#include "nrdc_show.h"
#include "Warp\Geometry2D.h"
#include "Warp\PCWarpSolverAdaptive.h"
#include "Warp\WarpRender.h"
#include "Preprocess\Preprocess.h"

#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace std;
using namespace LibIV;

// 给定一个mask和superpixel分类，求出该mask内的superpixel的index
// should be optimized
void findMaskedSuperpixels(const cv::Mat& unKnownMaskImg, const std::vector<std::vector<v2i>>& SP, std::vector<size_t>& maskedSuperpixels)
{
    maskedSuperpixels.clear();

#pragma  omp parallel for num_threads(8)
    for (int i = 0; i < SP.size(); ++i)
    {
        int pixelNum = 0;
        for (int j = 0; j < SP[i].size(); ++j)
        {
            v2i pos = SP[i][j];
            int p;
            LibIV::CppHelpers::Global::imgData(p, unKnownMaskImg, pos[0], pos[1]);
            if (p > 128)
            {
                pixelNum++;
            }
        }

        if (pixelNum > 0.5 * SP[i].size())
        {
#pragma omp critical
            {
                maskedSuperpixels.push_back(i);
            }

        }
    }
}

void NRDCProcessing::ConvertAscii2Binary(const char * filename1, const char * filename2)
{
	ifstream infile(filename1);
	ofstream outfile(filename2,ios::binary);

	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	string line;
	while(getline(infile,line))
	{
		istringstream iss(line);
		double d; 
		while(iss>>d)
		{
			bar.d = d;
			outfile.write(bar.c,sizeof(double));
		}
	}
	
	infile.close();
	outfile.close();
}

void NRDCProcessing::ReadNRDC(NRDC & nrdc, const char * filename)
{
	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	ifstream infile(filename,ios::binary);
	
	infile.seekg(0,ios::end);
    long long int ss = infile.tellg();
    char * buf = new char[(unsigned int)ss];
    infile.seekg(0,ios::beg);
    infile.read(buf,ss);
    char * p = buf;

	nrdc.fill(_v3d_(0,0,0));
	for(uint j = 0;j<nrdc.cols();j++)
	{
		for(uint i = 0;i<nrdc.rows();i++)
		{
			double d[5];
			for(int k = 0;k<5;k++)
			{
				memcpy(bar.c,p,sizeof(double));
				p+=sizeof(double);
				d[k] = bar.d;
			}

			if(d[0] != j+1 && d[1] != i+1)
				cout<<"ReadNRDC - error 1"<<endl;

			nrdc.at(i,j) = _v3d_(d[2],d[3],d[4]);
		}
	}

	//cout<<nrdc.at(300,300)[0]<<","<<nrdc.at(300,300)[1]<<","<<nrdc.at(300,300)[2]<<endl;

	delete [] buf;
}

void NRDCProcessing::ReadSP(const char * filename,std::vector<std::vector<v2i>> & SP)
{
	SP.clear();
	
	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	ifstream infile(filename,ios::binary);
	
	infile.seekg(0,ios::end);
    long long int ss = infile.tellg();
    char * buf = new char[(unsigned int)ss];
    infile.seekg(0,ios::beg);
    infile.read(buf,ss);
    char * p = buf;

	memcpy(bar.c,p,sizeof(double));
	int width = (int)(bar.d);
	p += sizeof(double);

	memcpy(bar.c,p,sizeof(double));
	int height = (int)(bar.d);
	p += sizeof(double);

	memcpy(bar.c,p,sizeof(double));
	int sp_num = (int)(bar.d);
	p+= sizeof(double);

	p+=sizeof(double); // skip 500

	SP.resize(sp_num);

	for(int i = 0;i<height;i++)
	{
		for(int j = 0;j<width;j++)
		{
			memcpy(bar.c,p,sizeof(double));
			p += sizeof(double);

			SP[(int)(bar.d)].push_back(_v2i_(j,i));
		}
	}

	delete [] buf;
}

void NRDCProcessing::BilinearInterpWeights(double x, double y, 
										   double & w00, double & w10, double & w01, double & w11)
{
	double xx = x - (int)(x);
	double yy = y - (int)(y);

	w00 = (1-xx)*(1-yy);
	w10 = xx*(1-yy);
	w01 = (1-xx)*yy;
	w11 = xx*yy;
}

bool NRDCProcessing::BilinearInterpImage(double x, double y, 
										 double & r, double & g, double & b, IplImage * img)
{
	int w = img->width;
	int h = img->height;

	r = g = b = 0;

	if(x < 0 || x > w-1 || y < 0 || y > h-1)
		return false;

	double wei[4];
	BilinearInterpWeights(x,y,wei[0],wei[1],wei[2],wei[3]);

	int j = (int)x;
	int i = (int)y;

	CvScalar cs[4];
	cs[0] = cvGet2D(img,i,j);
	cs[1] = cvGet2D(img,i,j+1);
	cs[2] = cvGet2D(img,i+1,j);
	cs[3] = cvGet2D(img,i+1,j+1);

	CvScalar ans = cvScalar(0);

	for(int i = 0;i<4;i++)
	{
		for(int j = 0;j<4;j++)
		{
			ans.val[i] += wei[j] * cs[j].val[i];
		}
	}

	r = ans.val[2];
	g = ans.val[1];
	b = ans.val[0];

	return true;
}

bool NRDCProcessing::BilinearInterpImage2(double x, double y, 
										 double & r, double & g, double & b, const cv::Mat & img)
{
	int w = img.cols;
	int h = img.rows;

	r = g = b = 0;

	if(x < 0 || x > w-1 || y < 0 || y > h-1)
		return false;

	double wei[4];
	BilinearInterpWeights(x,y,wei[0],wei[1],wei[2],wei[3]);

	int j = (int)x;
	int i = (int)y;

	int rr[4],gg[4],bb[4];

	LibIV::CppHelpers::Global::imgData(rr[0],gg[0],bb[0],img,j,i);
	LibIV::CppHelpers::Global::imgData(rr[1],gg[1],bb[1],img,j+1,i);
	LibIV::CppHelpers::Global::imgData(rr[2],gg[2],bb[2],img,j,i+1);
	LibIV::CppHelpers::Global::imgData(rr[3],gg[3],bb[3],img,j+1,i+1);
	
	r = (wei[0] * rr[0] + wei[1] * rr[1] + wei[2] * rr[2] + wei[3] * rr[3]);
	g = (wei[0] * gg[0] + wei[1] * gg[1] + wei[2] * gg[2] + wei[3] * gg[3]);
	b = (wei[0] * bb[0] + wei[1] * bb[1] + wei[2] * bb[2] + wei[3] * bb[3]);

	return true;
}

CvScalar NRDCProcessing::ImageGet2D(IplImage * img, double x, double y)
{
	double r,g,b;
	BilinearInterpImage(x,y,r,g,b,img);

	CvScalar cs;
	cs.val[0] = b;
	cs.val[1] = g;
	cs.val[2] = r;

	return cs;
}

void NRDCProcessing::ImageGet2D(int & r, int & g, int & b, const cv::Mat & img, double x, double y)
{
	double rr,gg,bb;
	BilinearInterpImage2(x,y,rr,gg,bb,img);

	r = (int)rr;
	g = (int)gg;
	b = (int)bb;
}

bool NRDCProcessing::ImageGet2D(IplImage * img, double x, double y, CvScalar & cs)
{
	double r,g,b;
	bool ok = BilinearInterpImage(x,y,r,g,b,img);

	cs.val[0] = b;
	cs.val[1] = g;
	cs.val[2] = r;

	return ok;
}

double NRDCProcessing::ComputeCoherenceBetweenPixels(v2i u, v2i v, const NRDC & nrdc)
{
	v3d uu = nrdc.at(u[1],u[0]);
	v3d vv = nrdc.at(v[1],v[0]);
	v2d pp;
	
	pp[0] = uu[0] - u[0] + v[0];
	pp[1] = uu[1] - u[1] + v[1];

	v2d v0,v1;

	v0[0] = vv[0] - pp[0];
	v0[1] = vv[1] - pp[1];

	v1[0] = uu[0] - pp[0];
	v1[1] = uu[1] - pp[1];

	double ans = sqrt(v0[0] * v0[0] + v0[1] * v0[1]) / sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
	return ans;
}

double NRDCProcessing::SPCoherence(const NRDC & nrdc, const std::vector<v2i> & sp)
{
	// srand(2028);
	// Z be the number of pixels in sp
	// z be the number of pixels with confidence greater than t_confidence
	// z/Z should be greater than t_sp_confidence

	double t_confidence = 0.3;  
	double t_sp_confidence = 0.5;   //0.5

	int z = 0;
	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];
		double c = nrdc.at(pos[1],pos[0])[2]; 
		if(c > t_confidence)
			z++;
	}
	
	if((double)(z)/(double)(sp.size()) <= t_sp_confidence)
	{
		return 1e10;
	}

    if (sp.size() < 200)
    {
        return 1e10;
    }

	//int JZ = (int)(sqrt((double)(sp.size())));
	int JZ = (int)(sp.size()) * 2;
	int num = 0;

	for(int i = 0;i<JZ;i++)
	{
		v2i u,v;

		while(true)
		{
			int i1 = rand() % sp.size();
			int i2 = rand() % sp.size();

			u = sp[i1];
			v = sp[i2];
				
			double x = u[0] - v[0];
			double y = u[1] - v[1];
			double dd = sqrt(x*x+y*y);
			if(dd > 4 && dd < 64) break;   // parameter
		}
		
		double c_bet_pix = ComputeCoherenceBetweenPixels(u,v,nrdc);
		//cout<<c_bet_pix<<endl;
		if(c_bet_pix > 2)   // parameter, 2
			num++;
	}

    double kkk = (double)(num)/(double)(JZ);
    if(kkk >= 0.7)   // [0, 1]
    {
        return 1e10;
    }

    /*
    // http://graphics.stanford.edu/~mdfisher/papers/patternColoring.pdf
    
    
    std::vector<v2d> spc;
    std::vector<int> is_inlier;
    cv::Mat H;
    NRDCProcessing::ComputeHomographyForSP(nrdc,sp,H);
    NRDCProcessing::HomographyTransform(sp,H,spc);
    is_inlier.resize(sp.size(),0);
    int inlier_num = 0;

    for(size_t i = 0;i<sp.size();i++)
    {
    v2i pos = sp[i];
    v2d posc = spc[i];
    v3d nrdcc = nrdc.at(pos[1],pos[0]);

    double xx = posc[0] - nrdcc[0];
    double yy = posc[1] - nrdcc[1];

    double dd = sqrt(xx * xx + yy * yy);
    if(dd < 5)
    {
    inlier_num++;
    is_inlier[i] = 1;
    }
    }

    double inlier_ratio = (double)inlier_num / (double)(sp.size());

    if(inlier_ratio < 0.6)
    {
    return 1e10;
    }

    double minx1 = 1e10,miny1=1e10,maxx1=-1e10,maxy1=-1e10;
    double minx2= 1e10,miny2=1e10,maxx2=-1e10,maxy2=-1e10;
    double cenx1,ceny1;
    double cenx2,ceny2;
    double ctodx1,ctody1;
    double ctodx2,ctody2;
    double ttx1=0,tty1 = 0,ttx2=0,tty2 = 0;

    for(size_t i = 0;i<sp.size();i++)
    {
    if(is_inlier[i] == 0) continue;

    v2i pos = sp[i];
    v2d posc = spc[i];

    if(pos[0] < minx1) minx1 = pos[0];
    if(pos[0] > maxx1) maxx1 = pos[0];
    if(pos[1] < miny1) miny1 = pos[1];
    if(pos[1] > maxy1) maxy1 = pos[1];

    if(posc[0] < minx2) minx2 = posc[0];
    if(posc[0] > maxx2) maxx2 = posc[0];
    if(posc[1] < miny2) miny2 = posc[1];
    if(posc[1] > maxy2) maxy2 = posc[1];

    ttx1 += pos[0];
    tty1 += pos[1];
    ttx2 += posc[0];
    tty2 += posc[1];
    }

    cenx1 = (maxx1 + minx1) / 2;
    ceny1 = (maxy1 + miny1) / 2;

    cenx2 = (maxx2 + minx2) / 2;
    ceny2 = (maxy2 + miny2) / 2;

    ctodx1 = ttx1 / sp.size();
    ctody1 = tty1 / sp.size();
    ctodx2 = ttx2 / sp.size();
    ctody2 = tty2 / sp.size();

    double xx1 = cenx1 - ctodx1;
    double yy1 = ceny1 - ctody1;

    double xx2 = cenx2 - ctodx2;
    double yy2 = ceny2 - ctody2;

    double dd1 = sqrt(xx1*xx1+yy1*yy1);
    double dd2 = sqrt(xx2*xx2 + yy2*yy2);

    double elong1 = 1 - (maxx1 - minx1) / (maxy1 - miny1);
    double elong2 = 1 - (maxx2 - minx2) / (maxy2 - miny2);

    if(abs(dd1-dd2) > 8 || abs(elong1 - elong2) > 1.0)
    {
    return 1e10;
    }*/

    return 0;
}

bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints,    
	const std::vector<cv::KeyPoint>& trainKeypoints,     
	float reprojectionThreshold,    
	std::vector<cv::DMatch>& matches,    
	cv::Mat& homography  )  
{  
	const int minNumberMatchesAllowed = 4;    
	if (matches.size() <minNumberMatchesAllowed)    
		return false;    
	// Prepare data for cv::findHomography    
	std::vector<cv::Point2f> queryPoints(matches.size());    
	std::vector<cv::Point2f> trainPoints(matches.size());    
	for (size_t i = 0; i <matches.size(); i++)    
	{    
		queryPoints[i] = queryKeypoints[matches[i].queryIdx].pt;    
		trainPoints[i] = trainKeypoints[matches[i].trainIdx].pt;    
	}    
	// Find homography matrix and get inliers mask    
	std::vector<unsigned char> inliersMask(matches.size());    
	homography = cv::findHomography(queryPoints,     
		trainPoints,     
		CV_FM_RANSAC,     
		reprojectionThreshold,     
		inliersMask);    
	std::vector<cv::DMatch> inliers;    
	for (size_t i=0; i<inliersMask.size(); i++)    
	{    
		if (inliersMask[i])    
			inliers.push_back(matches[i]);    
	}    
	matches.swap(inliers);  
	return matches.size() > minNumberMatchesAllowed;   

}  

void NRDCProcessing::ComputeHomographyForSP(const NRDC & nrdc, const std::vector<v2i> & sp, cv::Mat & H)
{
	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

	double dRatio;
	//// begin indicate homography
	//obj.push_back(cv::Point2d(451, 611));
	//scene.push_back(cv::Point2d(316, 725));

	//obj.push_back(cv::Point2d(661, 617));
	//scene.push_back(cv::Point2d(768, 743));

	//obj.push_back(cv::Point2d(531, 546));
	//scene.push_back(cv::Point2d(521, 548));

	//obj.push_back(cv::Point2d(605, 545));
	//scene.push_back(cv::Point2d(615, 550));

	//obj.push_back(cv::Point2d(698, 560));
	//scene.push_back(cv::Point2d(752, 580));

	//obj.push_back(cv::Point2d(362, 587));
	//scene.push_back(cv::Point2d(192, 639));

	//H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );

	// end indicate homography


	// begin compute homography using sift detector
#if 0
	std::string src_image_path = "F:/LastExperiments/Case28/forward/a.bmp";
	std::string tgt_image_path = "F:/LastExperiments/Case28/forward/b.bmp";

	cv::Mat src_image = cv::imread(src_image_path, CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat tgt_image = cv::imread(tgt_image_path, CV_LOAD_IMAGE_GRAYSCALE);

	if (!src_image.data || !tgt_image.data)
	{
		std::cout << "READ IMAGE ERROR!" << std::endl;
		return;
	}

	cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    detector.detect(src_image, keypoints1);
	detector.detect(tgt_image, keypoints2);

	/// load mask
	std::string mask_forward_ground = "F:/LastExperiments/Case28/forward/mask/mask_homo.bmp";
	std::string mask_forward_ground_back = "F:/LastExperiments/Case28/forward/mask/mask_homo_back.bmp";
	
	cv::Mat mask_fg_img = cv::imread(mask_forward_ground, CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat mask_fg_back_img = cv::imread(mask_forward_ground_back, CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<cv::KeyPoint> keypoints1_g, keypoints2_g;
	keypoints1_g.clear();
	keypoints2_g.clear();

	for (int i = 0; i < keypoints1.size(); i++)
	{
		int p = 0;
		LibIV::CppHelpers::Global::imgData(p, mask_fg_img, keypoints1[i].pt.x, keypoints1[i].pt.y);

		if(p > 128)
			keypoints1_g.push_back(keypoints1[i]);
	}

	for (int i = 0; i < keypoints2.size(); i++)
	{
		int p = 0;
		LibIV::CppHelpers::Global::imgData(p, mask_fg_back_img, keypoints2[i].pt.x, keypoints2[i].pt.y);

		if(p > 128)
			keypoints2_g.push_back(keypoints2[i]);
	}

	cv::SiftDescriptorExtractor extractor;
	cv::Mat desc1, desc2;
	extractor.compute(src_image, keypoints1_g, desc1);
	extractor.compute(tgt_image, keypoints2_g, desc2);

	const bool bUseKnnMatch = true;

	if(bUseKnnMatch)
	{
		// begin knn-match
		cv::BFMatcher matcher(cv::NORM_L2, false);

		std::vector<std::vector<cv::DMatch> > matches;
		matcher.knnMatch(desc1, desc2, matches, 2);

		std::vector<cv::DMatch> good_matches;
		good_matches.clear();

		const double dThreshold = 0.75;  //0.75

		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i][0].distance / matches[i][1].distance < dThreshold)
			{
				good_matches.push_back(matches[i][0]);
			}
		}

		float homoReprojThres = 3.0;
		bool homoFound = refineMatchesWithHomography(  
			keypoints1_g, keypoints2_g, homoReprojThres, good_matches, H);  

		std::cout << "Found homography? " << homoFound << std::endl;

		////绘制匹配线段
		cv::Mat img_matches;
		cv::drawMatches(src_image, keypoints1_g, tgt_image, keypoints2_g, good_matches, img_matches, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);//将匹配出来的结果放入内存img_matches中

		
		cv::imwrite("D:/hellobbb/matches_knn.bmp", img_matches);

	}
	else
	{
		// begin bf-match
		cv::BFMatcher matcher(cv::NORM_L2, true);

		std::vector<cv::DMatch> matches;
		matcher.match(desc1, desc2, matches);

		double max_dist = 0; double min_dist = 1000;
		for( int i = 0; i < matches.size(); i++ )
		{ 
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}

		std::cout << "-- Max dist: " << max_dist << std::endl;
		std::cout << "-- Min dist: " << min_dist << std::endl;

		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector< cv::DMatch > good_matches;

		for( int i = 0; i < matches.size(); i++ )
		{ 
			if( matches[i].distance < 2*min_dist )
			{ 
				good_matches.push_back( matches[i]); 
			}
		}

		obj.clear();
		scene.clear();

		for (int i = 0; i < good_matches.size(); i++)
		{
			int idx0 = good_matches[i].queryIdx;
			int idx1 = good_matches[i].trainIdx;

			obj.push_back(keypoints1_g[idx0].pt);
			scene.push_back(keypoints2_g[idx1].pt);
		}

		H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );

		////绘制匹配线段
		cv::Mat img_matches;
		cv::drawMatches(src_image, keypoints1_g, tgt_image, keypoints2_g, good_matches, img_matches, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);//将匹配出来的结果放入内存img_matches中

		
		cv::imwrite("D:/hellobbb/matches_bf.bmp", img_matches);

		//显示匹配线段
		//cv::namedWindow("sift matches");
		//cv::imshow("sift matches",img_matches);//显示的标题为Matches
		//cv::waitKey(0);
	}
#endif

	//std::cout << "new homography " << H << std::endl;


	//cv::SVD thissvd(H,cv::SVD::FULL_UV);
	//cv::Mat w = thissvd.w; 

	//dRatio = w.at<double>(0) / w.at<double>(2);
	//std::cout << "The ratio is " << dRatio << std::endl;

	

	// end compute homography using sift detector

	//return;



	// begin shuffle version
	//std::vector<int> index;
	//for(int i = 0;i<sp.size();i++)
	//	index.push_back(i);

	//do
	//{
	//	//std::next_permutation(index.begin(),index.end());
	//	std::random_shuffle(index.begin(),index.end());

	//	for(size_t i = 0;i<index.size();i++)
	//	{
	//		int idx = index[i];
	//		v2i pos = sp[idx];
	//		v3d pos2 = nrdc.at(pos[1],pos[0]);

	//		obj.push_back(cv::Point2d((double)(pos[0]),double(pos[1])));
	//		scene.push_back(cv::Point2d(pos2[0],pos2[1]));
	//	}

	//	H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );

	//	cv::SVD thissvd(H,cv::SVD::FULL_UV);
	//	cv::Mat w = thissvd.w; 

	// 	dRatio = w.at<double>(0) / w.at<double>(2);
	//	std::cout << dRatio << std::endl;

	//}while(dRatio > 1e7);

	// end shuffle version

	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];
		v3d pos2 = nrdc.at(pos[1],pos[0]);

		obj.push_back(cv::Point2d((double)(pos[0]),double(pos[1])));
		scene.push_back(cv::Point2d(pos2[0],pos2[1]));
	}

	H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC, 3 );
	
	

//	H = cv::findHomography(obj,scene,CV_RANSAC);
}

void NRDCProcessing::ComputeHomographyForSP(const NRDC & nrdc, const SuperPixels & SP, const std::vector<int> & seeds, cv::Mat & H)
{
	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

    //#pragma omp parallel for
	for(int j = 0;j<seeds.size();j++)
	{
	
		for(int i = 0;i<SP[seeds[j]].size();i++)
		{
            if(i%3 != 0)
                continue;

			v2i pos = SP[seeds[j]][i];
			v3d pos2 = nrdc.at(pos[1],pos[0]);

			obj.push_back(cv::Point2d((double)(pos[0]),double(pos[1])));
			scene.push_back(cv::Point2d(pos2[0],pos2[1]));
		}
	}
	H = cv::findHomography( cv::Mat(obj), cv::Mat(scene), CV_RANSAC );
}
void NRDCProcessing::HomographyTransform(const std::vector<v2i> & sp, cv::Mat & H, std::vector<v2d> & spc)
{
	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

	for(size_t i = 0;i<sp.size();i++)
		obj.push_back(cv::Point2d(sp[i][0],sp[i][1]));

	scene.resize(obj.size());

	cv::perspectiveTransform( cv::Mat(obj), cv::Mat(scene), H);
	//cv::perspectiveTransform(obj,scene,H);		

	spc.clear();
	for(size_t i = 0;i<scene.size();i++)
		spc.push_back(_v2d_(scene[i].x,scene[i].y));
}

double NRDCProcessing::ComputeDifferenceBetweenSPandSPC(IplImage * img, IplImage * imgc, 
														const std::vector<v2i> & sp, 
														const std::vector<v2d> & spc)
{
	double ans = 0;

	for(size_t i = 0;i<sp.size();i++)
	{
		v2i p = sp[i];
		v2d pc = spc[i];
		
		CvScalar cs = cvGet2D(img,p[1],p[0]);
		CvScalar csc; 
		bool flag = NRDCProcessing::ImageGet2D(imgc,pc[0],pc[1],csc);
		
		if(!flag)
			ans += 1e40;
		else
		{
			double db = cs.val[0] - csc.val[0];
			double dg = cs.val[1] - csc.val[1];
			double dr = cs.val[2] - csc.val[2];
			
			ans += db*db + dg*dg + dr*dr;
		}
	}
	
	return (sqrt(ans)/sp.size());
}

void NRDCProcessing::FindSupportRegionGlobalTraversal(IplImage * img_src, IplImage * img_dst, 
													  const NRDC & nrdc, 
													  const std::vector<std::vector<v2i>> & SP, 
													  int sp_idx, 
													  std::vector<int> & sp_support,
													  std::vector<double> & sp_support_diff)
{
	cv::Mat H;
	std::vector<v2d> spc;
	double base_diff;

	sp_support.clear();
	sp_support_diff.clear();
	
	
	NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);
	NRDCProcessing::HomographyTransform(SP[sp_idx],H,spc);
	base_diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img_src,img_dst,SP[sp_idx],spc);
	
	sp_support.push_back(sp_idx);
	sp_support_diff.push_back(base_diff);
	
	base_diff *= 1.1;
	
	// check all the other sps
	for(size_t i = 0;i<SP.size();i++)
	{
		if(i != sp_idx)
		{
			spc.clear();
			NRDCProcessing::HomographyTransform(SP[i],H,spc);
			double diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img_src,img_dst,SP[i],spc);
			if(diff < base_diff)
			{
				sp_support.push_back((int)i);
				sp_support_diff.push_back(diff);
			}
		}
	}
}

SpGraph * NRDCProcessing::MakeSpGraph(const std::vector<std::vector<v2i>> & SP, int width, int height)
{
	// 2d array of sp labels
	LibIV::Memory::Array::FastArray2D<int> sp_label_array;
	sp_label_array.set(width,height);
	sp_label_array.fill(0);

	for(size_t i = 0;i<SP.size();i++)
	{
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			sp_label_array.at(pos[1],pos[0]) = (int)i;
		}
	}

	// new SpGraph
	SpGraph * ans = new SpGraph();
	for(size_t i = 0;i<SP.size();i++)
	{
		SpNode * node = new SpNode();
		node->neighbors.clear();
		node->sp_id = (int)i;
		
		ans->push_back(node);
	}

	// compute neighbors
	for(size_t i = 0;i<ans->size();i++)
	{
		SpNode * node = (*ans)[i];
		int		 id   = node->sp_id;
		
		std::set<int> nid;
		for(size_t j = 0;j<SP[id].size();j++)
		{
			v2i pos = SP[id][j];
			
			int x = pos[0];
			int y = pos[1];

			if(x > 0)
			{
				int b = sp_label_array.at(y,x-1);
				if(b != id)
					nid.insert(b);
			}
			if(x < width - 1)
			{
				int b = sp_label_array.at(y,x+1);
				if(b != id)
					nid.insert(b);
			}
			if(y > 0)
			{
				int b = sp_label_array.at(y-1,x);
				if(b != id)
					nid.insert(b);
			}
			if(y < height - 1)
			{
				int b = sp_label_array.at(y+1,x);
				if(b != id)
					nid.insert(b);
			}
		}

		std::set<int>::iterator iter;
		for(iter = nid.begin(); iter!=nid.end();++iter)
			node->neighbors.push_back(*iter);
	}

	NRDCProcessing::ClearGraphTmp(ans);

	return ans;
}

void NRDCProcessing::DeleteSpGraph(SpGraph ** graph)
{
	for(size_t i = 0;i<(*graph)->size();i++)
		delete (**graph)[i];

	graph = 0;
}

void NRDCProcessing::ClearGraphTmp(SpGraph * graph)
{
	for(size_t i = 0;i<graph->size();i++)
	{
		(*graph)[i]->tmp0 = 0;
		(*graph)[i]->tmp1 = 0;
		(*graph)[i]->tmp2 = 0;
		(*graph)[i]->child_nodes.clear();
		(*graph)[i]->parent_nodes.clear();
		(*graph)[i]->edge_use_info.clear();
		(*graph)[i]->edge_use_info.resize((*graph)[i]->neighbors.size(),0);
	}
}

void NRDCProcessing::CollectN_Neighbors(SpGraph * graph, int sp_idx, int N, std::vector<v2d> & neighbors)
{
	NRDCProcessing::ClearGraphTmp(graph);
	neighbors.clear();

	std::queue<SpNode*> que;
	que.push((*graph)[sp_idx]);
	(*graph)[sp_idx]->tmp0 = 1;
	
	while(!que.empty())
	{
		SpNode * node = que.front();
		que.pop();

		neighbors.push_back(_v2d_(node->sp_id,node->tmp1));
		if(node->tmp1 >= N) continue;

		for(size_t i = 0;i<node->neighbors.size();i++)
		{
			SpNode * nnode = (*graph)[node->neighbors[i]];
			if(nnode->tmp0 != 1)
			{
				nnode->tmp1 = node->tmp1 + 1;
				que.push(nnode);
				nnode->tmp0 = 1;
			}
		}
	}
}

void NRDCProcessing::FindSupportRegionWithinN_Neighbors(IplImage * img_src, IplImage * img_dst, 
														const NRDC & nrdc, 
														const std::vector<std::vector<v2i>> & SP, 
														SpGraph * graph, int sp_idx, 
														std::vector<v2d> & sp_support, cv::Mat & H)
{
	sp_support.clear();

	// find N neighbors
	std::vector<v2d> N_neighbors;
	NRDCProcessing::CollectN_Neighbors(graph,sp_idx,5,N_neighbors); // 5 - ring - neighbors
	
	// compute Homography and base_diff
	double base_diff;
	std::vector<v2d> spc;
	NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);
	NRDCProcessing::HomographyTransform(SP[sp_idx],H,spc);
	base_diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img_src,img_dst,SP[sp_idx],spc);
	sp_support.push_back(_v2d_(sp_idx,base_diff));
	
	//base_diff *= 1.1;

	// check each neighbor if they support current sp
	for(size_t i = 1;i<N_neighbors.size();i++)
	{
		int	sp_idx_neighbor = (int)(N_neighbors[i][0]);
		double dist = N_neighbors[i][1];
		spc.clear();
		NRDCProcessing::HomographyTransform(SP[sp_idx_neighbor],H,spc);
		double diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img_src,img_dst,SP[sp_idx_neighbor],spc);

		//double adaptive_diff = base_diff * 1.1 * exp(-0.5 * dist * dist / (2*2));
		//double adaptive_diff = base_diff * 1;
		//if(diff < adaptive_diff)
			sp_support.push_back(_v2d_(sp_idx_neighbor,diff));
	}

	// finally, isolate supprot region
	NRDCProcessing::IsolateSupportRegion(graph,sp_support);
}

void NRDCProcessing::FindSupportRegionWithinN_Neighbors(IplImage * img_src, IplImage * img_dst, 
														const NRDC & nrdc, 
														const std::vector<std::vector<v2i>> & SP, 
														int sp_idx, 
														std::vector<v2d> & sp_support, cv::Mat & H)
{
	sp_support.clear();
	
	// compute Homography and base_diff
	double base_diff;
	std::vector<v2d> spc;
	NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);
	NRDCProcessing::HomographyTransform(SP[sp_idx],H,spc);
	base_diff = NRDCProcessing::ComputeDifferenceBetweenSPandSPC(img_src,img_dst,SP[sp_idx],spc);
	sp_support.push_back(_v2d_(sp_idx,base_diff));
}

void NRDCProcessing::IsolateSupportRegion(SpGraph * graph, std::vector<v2d> & sp_support)
{
	if(sp_support.empty()) return;
	std::vector<v2d> sps;

	NRDCProcessing::ClearGraphTmp(graph);

	// mark nodes in the support region
	for(size_t i = 0;i<sp_support.size();i++)
	{
		(*graph)[(int)(sp_support[i][0])]->tmp0 = 1;
		(*graph)[(int)(sp_support[i][0])]->tmp1 = sp_support[i][1];
	}

	// breadth-width traverse
	std::queue<SpNode*> que;
	que.push((*graph)[(int)(sp_support[0][0])]);
	(*graph)[(int)(sp_support[0][0])]->tmp2 = 1;
	
	while(!que.empty())
	{
		SpNode * node = que.front();
		que.pop();
		sps.push_back(_v2d_(node->sp_id,node->tmp1));

	
		for(size_t i = 0;i<node->neighbors.size();i++)
		{
			SpNode * nnode = (*graph)[node->neighbors[i]];
			if(nnode->tmp0 == 1 && nnode->tmp2 != 1)
			{
				que.push(nnode);	
				nnode->tmp2 = 1;
			}
		}
	}
	
	sp_support.clear();
	for(size_t i = 0;i<sps.size();i++)
		sp_support.push_back(sps[i]);
}

void NRDCProcessing::ViewInterpolation(const std::vector<int> & seeds, 
									   IplImage * img_src, IplImage * img_dst, 
									   const NRDC & nrdc, 
									   const std::vector<std::vector<v2i>> & SP, 
									   SpGraph * graph)
{
	cout<<"Compute correspondence"<<endl;

	std::vector<std::vector<v2d>> local_planes;
	std::vector<cv::Mat> local_homos;

	for(size_t i = 0;i<seeds.size();i++)
	{
		if(i%50 == 0)cout<<i<<endl;
		
		std::vector<v2d> sp_support;
		cv::Mat H;

		/*// have support region
		NRDCProcessing::FindSupportRegionWithinN_Neighbors(img_src,img_dst,
			nrdc,
			SP,
			graph,
			seeds[i],
			sp_support,
			H);*/

		// no supprot region, just the sp itself
		NRDCProcessing::FindSupportRegionWithinN_Neighbors(img_src,img_dst,
			nrdc,
			SP,
			seeds[i],
			sp_support,
			H);
			
		local_planes.push_back(sp_support);
		local_homos.push_back(H);
	}
	
	cout<<"Interplation"<<endl;

	for(int i = 0;i<100;i++)
	{
		cout<<i<<endl;

		double t = i / 100.0;
		IplImage * img_view = cvCreateImage(cvSize(img_src->width,img_src->height),8,3);
		cvZero(0);

		for(size_t j = 0;j<seeds.size();j++)
		{
			cv::Mat H = local_homos[j];
			std::vector<v2d> sp_support = local_planes[j];
			for(size_t k = 0;k<sp_support.size();k++)
			{
				int sp_idx = (int)(sp_support[k][0]);
				std::vector<v2d> spc, spcc;
				NRDCProcessing::HomographyTransform(SP[sp_idx],H,spc);

				//complex method can be replaced
				for(size_t kk = 0;kk<spc.size();kk++)
				{
					v2i pos = SP[sp_idx][kk];
					v2d posc = spc[kk];

					double x = (posc[0] - pos[0])*t + pos[0];
					double y = (posc[1] - pos[1])*t + pos[1];
					
					spcc.push_back(_v2d_(x,y));
				}

				NRDCShow::ShowSPc(img_src,img_view,SP[sp_idx],spcc);

				/*//simple method can be replaced

				for(size_t kk = 0;kk<spc.size();kk++)
				{
					v2i pos = SP[sp_idx][kk];
					v2d posc = spc[kk];

					int x = (int)((posc[0] - pos[0])*t + pos[0]);
					int y = (int)((posc[1] - pos[1])*t + pos[1]);
					
					if(x >= 0 && y >= 0
						&& x < img_src->width
						&& y < img_dst->height)
						//cvSet2D(img_view,y,x,cvGet2D(img_src,pos[1],pos[0]));
					{
						int r,g,b;
						LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
						LibIV::CppHelpers::Global::setImgData(r,g,b,img_view,x,y);
					}
				}*/
			}
		}
		
		char filename[1024];
		sprintf_s(filename,1024,"%s/viewinterp/%d.png",NRDCTest::folerpath.c_str(),i);
		cvSaveImage(filename,img_view);
		cvReleaseImage(&img_view);
	}
}

void NRDCProcessing::InitSeeds(std::vector<int> & seeds, 
							   const std::vector<std::vector<v2i>> & SP, 
							   const NRDC & nrdc, int width, int height)
{
	/*LibIV::Memory::Array::FastArray2D<int> sp_label_array;
	sp_label_array.set(width,height);
	sp_label_array.fill(0);

	for(size_t i = 0;i<SP.size();i++)
	{
		for(size_t j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			sp_label_array.at(pos[1],pos[0]) = i;
		}
	}

	for(int i = 40;i<height;i+=100)
		for(int j = 40;j<width;j+=100)
			seeds.push_back(sp_label_array.at(i,j));*/
	
	/*for(int i = 0;i<488;i++)
		seeds.push_back(i);*/

	// before add a seed, check it first
	
	for(int i = 0;i<485;i++)
	{
		
		double co = NRDCProcessing::SPCoherence(nrdc,SP[i]);
		// box: 0.01
		// castle:
		if(i == 430)
			cout<<co<<endl;
	
		if(co<=0.1)
			seeds.push_back(i);
	}

	/*seeds.push_back(349);
	seeds.push_back(350);
	seeds.push_back(381);
	seeds.push_back(402);
	seeds.push_back(406);
	seeds.push_back(355);
	seeds.push_back(363);
	seeds.push_back(367);
	seeds.push_back(366);
	seeds.push_back(382);
	seeds.push_back(387);
	seeds.push_back(380);
	seeds.push_back(388);
	seeds.push_back(409);
	seeds.push_back(405);
	seeds.push_back(416);
	seeds.push_back(412);
	seeds.push_back(408);
	seeds.push_back(411);
	seeds.push_back(441);
	seeds.push_back(445);
	seeds.push_back(431);
	seeds.push_back(451);*/

	/*seeds.push_back(405);
	seeds.push_back(416);
	seeds.push_back(381);*/
}

void NRDCProcessing::ComputeHistogramForSPs(const cv::Mat & img, const SuperPixels & SP, Histograms & hists)
{
	for(size_t sp_idx = 0; sp_idx < SP.size(); sp_idx++)
	{
		Histogram r_hist,g_hist,b_hist;
		NRDCProcessing::ComputeHistogramForSpRgb(img,SP,(int)sp_idx,r_hist,g_hist,b_hist);
		hists.push_back(r_hist);
		hists.push_back(g_hist);
		hists.push_back(b_hist);
	}

	/*cv::Mat img_lab;
	cv::cvtColor(img,img_lab,CV_BGR2Lab);

	for(size_t sp_idx = 0;sp_idx<SP.size();sp_idx++)
	{
		Histogram hist;
		NRDCProcessing::ComputeHistogramForSpLab(img_lab,SP,(int)sp_idx,hist);
		hists.push_back(hist);
	}*/
}

void NRDCProcessing::ComputeHistogramForSpLab(const cv::Mat & img_lab, 
											  const SuperPixels & SP, int sp_idx, Histogram & hist)
{
	const int lum_bin = 20;
	const int color_bin = 20;
	double weight_sum = 0;

	hist.clear();
	hist.resize(lum_bin + color_bin * 2);
	
	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		int L,a,b;
		LibIV::CppHelpers::Global::imgData(b,a,L,img_lab,pos[0],pos[1]);
		
		hist[(L * lum_bin) >> 8]++;
		hist[lum_bin + ((a * color_bin) >> 8)]++;
		hist[lum_bin + color_bin + ((b * color_bin) >> 8)]++;
		
		weight_sum += 3;
	}

	for(size_t i = 0;i<hist.size();i++)
		hist[i]/=weight_sum;
	
	/*cv::Mat img_show(60,60,CV_8UC3,cv::Scalar(0,0,0));
	for(size_t i = 0;i<hist.size();i++)
	{
		cv::line(img_show,cv::Point(i,59),cv::Point(i,(int)((1-hist[i]) * 59)),cv::Scalar(255,0,0));
	}
	cv::imwrite("E:/testsss.png",img_show);*/
}

void NRDCProcessing::ComputeHistogramForSpRgb(const cv::Mat & img, 
											  const SuperPixels & SP, int sp_idx, 
											  Histogram & r_hist, Histogram & g_hist, Histogram & b_hist)
{
	int r_bin,g_bin,b_bin;
	r_bin = g_bin = b_bin = 16;  // 16个bin
	
	r_hist.clear();
	g_hist.clear();
	b_hist.clear();

	r_hist.resize(r_bin);
	g_hist.resize(g_bin);
	b_hist.resize(b_bin);
	
	double weight_sum = 0;

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		
		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img,pos[0],pos[1]);
		
		r_hist[(r * r_bin) >> 8]+=1;  // 这里实际上是 r_hist[r / 256 * 16]+=1;
		g_hist[(g * g_bin) >> 8]+=1;
		b_hist[(b * b_bin) >> 8]+=1;
		
		weight_sum+=1;
	}

	for(int i = 0;i<r_bin;i++)
		r_hist[i] /= weight_sum;
	for(int i = 0;i<g_bin;i++)
		g_hist[i] /= weight_sum;
	for(int i = 0;i<b_bin;i++)
		b_hist[i] /= weight_sum;
}

double NRDCProcessing::ChiSquareDistBtwHist(const Histogram & lhs_hist, const Histogram & rhs_hist)
{
    // Please refer to http://www.researchgate.net/post/What_is_chi-squared_distance_I_need_help_with_the_source_code
	double ans = 0;
	for(size_t i = 0;i<lhs_hist.size();i++)
	{
		double a = lhs_hist[i];

		double b = rhs_hist[i];

		double ab_add = a + b;
		double ab_sub = a - b;
		
		if(ab_add > 1e-12)
			ans += (ab_sub * ab_sub / ab_add);
		else
			ans += 0;
	}

	return 0.5 * ans;
}

double NRDCProcessing::ChiSquareDistBtwSp(const Histograms & hists, int lhs_sp_idx, int rhs_sp_idx)
{
	Histogram l_r_hist,l_g_hist,l_b_hist,
			r_r_hist,r_g_hist,r_b_hist;

	l_r_hist = hists[lhs_sp_idx * 3];
	l_g_hist = hists[lhs_sp_idx * 3 + 1];
	l_b_hist = hists[lhs_sp_idx * 3 + 2];

	r_r_hist = hists[rhs_sp_idx * 3];
	r_g_hist = hists[rhs_sp_idx * 3 + 1];
	r_b_hist = hists[rhs_sp_idx * 3 + 2];

	double rchi = NRDCProcessing::ChiSquareDistBtwHist(l_r_hist,r_r_hist);
	double gchi = NRDCProcessing::ChiSquareDistBtwHist(l_g_hist,r_g_hist);
	double bchi = NRDCProcessing::ChiSquareDistBtwHist(l_b_hist,r_b_hist);

	return (rchi+gchi+bchi);
}

void NRDCProcessing::ChiSquareDistOfEdge(SpGraph * graph, const Histograms & hists, SpChiDiff & sp_chi_diff)
{
	int sp_num = (int)(graph->size());
	
	sp_chi_diff.set(sp_num,sp_num);
	sp_chi_diff.fill(-1);

	for(int i = 0;i<sp_num;i++)
	{
		SpNode * node = (*graph)[i];
		
		for(size_t j = 0;j<node->neighbors.size();j++)
		{
			int id1 = node->sp_id;
			int id2 = node->neighbors[j];
			
			sp_chi_diff.at(id1,id2) = ChiSquareDistBtwSp(hists,id1,id2);
		}
	}
}

void NRDCProcessing::DijkstraShortestPath(SpGraph * graph, const SpChiDiff & sp_chi_diff, 
										  int source_sp_idx, SpNode *& path_tree)
{
	NRDCProcessing::ClearGraphTmp(graph);

	std::vector<SpNode*> boundary_nodes;
	std::vector<SpNode*> unknown_nodes;
	
	boundary_nodes.push_back((*graph)[source_sp_idx]);
	// when add a node into boundary set, all paths from neighboring nodes to the node can not be 
	// considered again, as the shortest path to the node is found
	for(size_t i = 0;i<boundary_nodes[0]->neighbors.size();i++)
	{
		SpNode * neigh_node = (*graph)[boundary_nodes[0]->neighbors[i]];
		
		for(size_t j = 0;j<neigh_node->neighbors.size();j++)
			if(neigh_node->neighbors[j] == boundary_nodes[0]->sp_id)
				neigh_node->edge_use_info[j] = 1;
	}

	for(int i = 0;i<(int)((*graph).size());i++)
		if(i!=source_sp_idx) 
			unknown_nodes.push_back((*graph)[i]);
	
	path_tree = boundary_nodes[0];

	while(!boundary_nodes.empty() && !unknown_nodes.empty())
	{
		// traverse the boundary nodes, find the one that has the minimum distance to 
		// its neighboring unknow nodes
		double min_dist = 1e10;
		SpNode * min_node = 0;
		int min_neigh = 0;

		for(size_t i = 0;i<boundary_nodes.size();i++)
		{
			SpNode * bnode = boundary_nodes[i];
			for(size_t j = 0;j<bnode->neighbors.size();j++)
			{
				if(bnode->edge_use_info[j] == 0)
				{
					int idx1 = bnode->sp_id;
					int idx2 = (*graph)[bnode->neighbors[j]]->sp_id;
					
					double chi = sp_chi_diff.at(idx1,idx2);

					double cur_dist = chi + bnode->tmp0;

					if(cur_dist < min_dist)
					{
						min_dist = cur_dist;
						min_node = bnode;
						min_neigh = (int)j;
					}
				}
			}
		}
		
		// add that unknown node into boundary node set, and do some handover things
		if(min_node != 0)
		{
			SpNode * next_add_node = (*graph)[min_node->neighbors[min_neigh]];
			//min_node->child_nodes.push_back(next_add_node);
			next_add_node->parent_nodes.push_back(min_node);
			next_add_node->tmp0 = min_dist;
			
			boundary_nodes.push_back(next_add_node);
			for(size_t i = 0;i<next_add_node->neighbors.size();i++)
			{
				SpNode * neigh_node = (*graph)[next_add_node->neighbors[i]];
		
				for(size_t j = 0;j<neigh_node->neighbors.size();j++)
					if(neigh_node->neighbors[j] == next_add_node->sp_id)
						neigh_node->edge_use_info[j] = 1;
			}
			
			// remove that unknown node out of unknown set
			for(std::vector<SpNode*>::iterator it = unknown_nodes.begin(); 
				it != unknown_nodes.end();)
			{
				if((*it) == next_add_node)
				{
					it = unknown_nodes.erase(it);
					break;
				}
				else
					++it;
			}
		}

		// traverse the boundary set, remove a node if its edges are all used 
		for(std::vector<SpNode*>::iterator it = boundary_nodes.begin();
			it != boundary_nodes.end();)
		{
			bool is_all_1 = true;
			for(size_t i = 0;i<(*it)->edge_use_info.size();i++)
				if((*it)->edge_use_info[i] != 1)
				{
					is_all_1 = false;
					break;
				}
			
			if(is_all_1)
				it = boundary_nodes.erase(it);
			else
				++it;
		}
	}
}

void NRDCProcessing::DijkstraShortestPath2(SpGraph * graph, const SpChiDiff & sp_chi_diff, 
										  int source_sp_idx, SpNode *& path_tree)
{
	
}

void NRDCProcessing::DivideSpCategory(SpGraph * graph, 
									  const SuperPixels & SP, const NRDC & nrdc, 
									  std::vector<int> & sp_cate_label)
									  
{
	NRDCProcessing::ClearGraphTmp(graph);

	sp_cate_label.clear();
	sp_cate_label.resize(graph->size());

	for(size_t i = 0;i<SP.size();i++)
	{
		if(i==271)
			double ddd= -1;
        // The smaller incoherence, the better the superpixel is.
		double co = NRDCProcessing::SPCoherence(nrdc,SP[i]);
	
		if(co <= 0.8) // this parameter should be tuned to adapt to different examples, default 0.8
		{			
			sp_cate_label[i] = 1;	 // coherence
			(*graph)[i]->tmp2 = 1;   
		}
		else
			sp_cate_label[i] = -1;   // not coherence, (*graph)[i]->tmp2 = 0; but 0 is default 
	}

	for(size_t i = 0;i<SP.size();i++)
	{
		SpNode * node = (*graph)[i];

		if(node->tmp2 == 1)
			for(size_t j = 0;j<node->neighbors.size();j++)
			{
				SpNode * neigh_node = (*graph)[node->neighbors[j]];
				if(neigh_node->tmp2 == 0)
				{
					sp_cate_label[i] = 0;   // 1 -> 0
					break;
				}
			}
	}
}


void NRDCProcessing::ComputeNearestBoundarySpForMissingSp(SpGraph * graph, const SpChiDiff & sp_chi_diff, const Histograms & hists, const std::vector<int> & sp_cate_label, const std::vector<int> & sp_mask, int missing_sp_idx, int & nearest_idx)
{
	SpNode * path_tree;
	NRDCProcessing::DijkstraShortestPath(graph,sp_chi_diff,missing_sp_idx,path_tree);
	
	int mak = sp_mask[missing_sp_idx];

	double near_dist = 1e90;
	for(size_t i = 0;i<sp_cate_label.size();i++)
	{
		if(sp_cate_label[i] == 0 && sp_mask[i] == mak) // on the boundary, and belong to the same mask
		{
			double chi1 = NRDCProcessing::ChiSquareDistBtwSp(hists,missing_sp_idx, (int)i);
			double chi2 = (*graph)[i]->tmp0;

            int len = 0;
            SpNode * node = (*graph)[i];
            while(node != 0)
            {
                len++;
                node = (node->parent_nodes.size() > 0) ? node->parent_nodes[0] : 0;
            }


            int r = 2;
            chi2 = chi2 / exp(-0.5 * (len * len) / (r * r));

            //std::cout << "chi1: " << chi1 << std::endl;
            //std::cout << "chi2: " << chi2 << std::endl;

			double chi = chi1 + chi2;
			if(chi < near_dist)
			{
				near_dist = chi;
				nearest_idx = (int)i;
			}
		}
	}
    //std::cout << "nearest distance: " << near_dist << std::endl;
}

bool less_function(const v2d & v1, const v2d & v2)
{
	return v1[1] < v2[1];
}

void NRDCProcessing::ComputeSeedsForHomographyInference(SpGraph * graph, const SpChiDiff & sp_chi_diff, 
														const std::vector<int> & sp_cate_label, 
														const std::vector<int> & sp_mask,
														int nearest_boundary_idx, 
														std::vector<int> & seeds)
{
	SpNode * path_tree;
	NRDCProcessing::DijkstraShortestPath(graph,sp_chi_diff,nearest_boundary_idx, path_tree);
	
	int mak = sp_mask[nearest_boundary_idx];

	double max_dist = 20;
	// find all super pixels with the distance less than max_dist
	for(size_t i = 0;i<graph->size();i++)
	{
		double tmp = (*graph)[i]->tmp0;
		if((*graph)[i]->tmp0 < max_dist 
			&& sp_cate_label[i] >= 0 // should be in coherent set
			&& sp_mask[i] == mak     // should have same mask
			) 
			seeds.push_back((int)i);
	}

	// the best seeds should have shortest path through missing region
	std::vector<v2d> score;
	score.resize(seeds.size(),_v2d_(0,0));

	for(size_t i = 0;i<seeds.size();i++)
	{
		SpNode * node = (*graph)[seeds[i]];
		
		int num_of_missing = 0;
		while(node != 0)
		{
			if(sp_cate_label[node->sp_id] == -1) 
				num_of_missing++;
			node = (node->parent_nodes.size() > 0) ? node->parent_nodes[0] : 0;
		}
		
		double s1 = exp(-0.5 * num_of_missing * num_of_missing / (3 * 3));
		//cout<<num_of_missing<<endl;
		double s2 = (*graph)[seeds[i]]->tmp0;
		//score[i] = _v2d_(seeds[i],s1 * s2);    // origin
        score[i] = _v2d_(seeds[i], s2);   // revision
	}

	// find the best five seeds that have minimum scores
	std::sort(score.begin(),score.end(),less_function);
	seeds.clear();

    int seedsNum = 5;
	
	int seed_size = score.size() > seedsNum ? seedsNum : (int)(score.size());
	for(int i = 0;i<seed_size;i++)
		seeds.push_back((int)(score[i][0]));
}

void NRDCProcessing::HistogramMatching(const cv::Mat & img_src, const SuperPixels & SP, 
									   const std::vector<int> & seeds, 
									   int sp_idx, 
									   std::vector<v2i> & samples)
{
	Histogram hsnorm;
	Histogram ht, htnorm;
	int bin_num = 16; 
	int total_num = bin_num * bin_num * bin_num;

	hsnorm.resize(total_num);
	ht.resize(total_num);
	htnorm.resize(total_num);

	std::vector<std::vector<v2i>> attached_pos;
	std::vector<std::vector<v2i>> to_loss_pos;
	attached_pos.resize(total_num);
	to_loss_pos.resize(total_num);

	int ws = 0, wt = 0;
	
	// compute a reference histogram
	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		
		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);

		int bin = ((r * bin_num)>>8) * bin_num * bin_num +
			((g * bin_num)>>8) * bin_num +
			((b * bin_num)>>8);

		hsnorm[bin]+=1;
		ws+=1;
	}

	for(int i = 0;i<total_num;i++)
		hsnorm[i] /= ws;

	// compute the adjusted histogram 
	for(size_t i = 0;i<seeds.size();i++)
	{
		for(size_t j = 0;j<SP[seeds[i]].size();j++)
		{
			v2i pos = SP[seeds[i]][j];

			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);

			int bin = ((r * bin_num)>>8) * bin_num * bin_num +
				((g * bin_num)>>8) * bin_num +
				((b * bin_num)>>8);

			ht[bin] += 1;
			wt+=1;
			
			attached_pos[bin].push_back(pos);
		}
	}

	// histogram matching
	int iter_num = 1000;
	while(iter_num-->0 && wt > 0)
	{
		// normlize ht
		for(int i = 0;i<total_num;i++)
			htnorm[i] = ht[i] / wt;
		
		// find the most different bin
		std::vector<double> bin_diff;
		bin_diff.resize(total_num);
		for(int i = 0;i<total_num;i++)
			bin_diff[i] = htnorm[i] - hsnorm[i];
		
		std::vector<double>::iterator iter = std::max_element(bin_diff.begin(),bin_diff.end());
		int max_diff_bin = (int)(iter - bin_diff.begin());
		double max_diff  = bin_diff[max_diff_bin];

		// equalize two histogram
		if(max_diff < 0) 
		{
			cout<<"max_diff < 0 - HistogramMatching"<<endl;
			break;
		}
		
		int num_pos_to_loss = (int)((max_diff / 2) * attached_pos[max_diff_bin].size());
		for(int i = 0;i<num_pos_to_loss;i++)
		{
			int rand_p = rand() % attached_pos[max_diff_bin].size();
			std::vector<v2i>::iterator iter = attached_pos[max_diff_bin].begin() + rand_p;
			to_loss_pos[max_diff_bin].push_back((*iter));
			attached_pos[max_diff_bin].erase(iter);
		}

		ht[max_diff_bin]-=num_pos_to_loss;
		wt -= num_pos_to_loss;
	}

	for(int i = 0;i<total_num;i++)
	{
		for(size_t j = 0;j<attached_pos[i].size();i++)
			samples.push_back(attached_pos[i][j]);
	}
}

void NRDCProcessing::LocalWarping(const cv::Mat & img_src, const SuperPixels & SP, 
								  const NRDC & nrdc, SpGraph * graph, 
								  const Histograms & hists, const SpChiDiff & sp_chi_diff, 
								  const std::vector<int> & sp_mask, const std::vector<int> & sp_cate_label, 
								  cv::Mat & img_local, 
								  std::vector<Triangle> & all_triangles, 
								  std::vector<Vertex> & all_vertices, 
								  std::vector<Vertex> & all_deformed_vertices)
{
}


double  NRDCProcessing::ComputeDisplacementOfSuperpixel (const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat propogatedH)
{
     SuperPixel csp = SP[sp_idx];
    
    /* center of csp */
    double xx = 0, yy = 0;
    
    for(size_t i = 0;i<csp.size();i++)
    {
        v2i cpt = csp[i];
        xx += cpt[0];
        yy += cpt[1];
    }

    xx /= (double)(csp.size());
    yy /= (double)(csp.size());
    

    /* transformed superpixel */
    std::vector<v2d> tsp;
    HomographyTransform(csp,propogatedH,tsp);

    /* center of tsp */
    double xxx = 0, yyy = 0;
    for(size_t i = 0;i<tsp.size();i++)
    {
        v2d cpt = tsp[i];
        xxx += cpt[0];
        yyy += cpt[1];
    }

    xxx /= (double)(tsp.size());
    yyy /= (double)(tsp.size());

    double dx = xx - xxx;
    double dy = yy - yyy;

    double dd = sqrt(dx * dx + dy * dy);
    return dd;
}


bool NRDCProcessing::CheckIfPropogatedHomographyReliable (const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat propogatedH, double thres)
{
    double dd = ComputeDisplacementOfSuperpixel(SP, sp_idx, propogatedH);
    
    return (dd < thres);
}


void NRDCProcessing::DrawHomoWarp(const NRDC & nrdc,const std::vector<v2i> & sp, cv::Mat & H, int spid, int width, int height)
{
	cv::Mat testImg(height, width, CV_8UC3, cv::Scalar(0));
	std::string imgPath = "D:/helloaaa/homos/" + std::to_string(spid) + ".png";

	std::vector<cv::Point2d> obj;
	std::vector<cv::Point2d> scene;

	for(size_t i = 0;i<sp.size();i++)
		obj.push_back(cv::Point2d(sp[i][0],sp[i][1]));

	scene.resize(obj.size());

	cv::perspectiveTransform( cv::Mat(obj), cv::Mat(scene), H);

	for (int i = 0; i < sp.size(); i++)
	{
		v2i pos = sp[i];
		LibIV::CppHelpers::Global::setImgData(255,255,255,testImg,pos[0],pos[1]);
		//std::cout << scene[i] << std::endl;

		if (scene[i].x > 0  && scene[i].y > 0 && scene[i].x < width && scene[i].y < height)
		{
			LibIV::CppHelpers::Global::setImgData(0,0,255,testImg,scene[i].x,scene[i].y);
		}
		
	}

	cv::imwrite(imgPath, testImg);

}


void NRDCProcessing::LocalWarping2(const cv::Mat & img_src, const SuperPixels & SP, 
								  const NRDC & nrdc, SpGraph * graph, 
								  const Histograms & hists, const SpChiDiff & sp_chi_diff, 
								  const std::vector<int> & sp_mask, const std::vector<int> & sp_cate_label, 
								  LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre,
								  int mask_id)
{
	std::cout << "LocalWarping begins ..." << std::endl << std::flush;

	std::cout << "sp idx classify ..." << std::endl << std::flush;
	std::vector<int> sp_idx_consider, sp_idx_missing, sp_idx_known;

	for(size_t i = 0;i<sp_mask.size();i++)
	{
		if(sp_mask[i] == 1)
		{
			sp_idx_consider.push_back((int)i);
			if(sp_cate_label[i] >= 0)
				sp_idx_known.push_back((int)i);
			else
				sp_idx_missing.push_back((int)i);
		}
	}

	cout<<"compute homographies..."<<std::endl<<std::flush;
	std::vector<cv::Mat> homoS;
	std::vector<cv::Mat> homoT;
    std::vector<int> validHomoS; // 0 - invalid, 1 - valid

	homoS.resize(graph->size(), cv::Mat::zeros(3,3,CV_64F));
	homoT.resize(graph->size(), cv::Mat::zeros(3,3,CV_64F));
    validHomoS.resize(graph->size(), 0);

    double t1, t2;
    t1 = omp_get_wtime();

    double dDisplacementThres = 0;

 

    // 可靠的superpixel, consistent
    //#pragma omp parallel for
	for(int i = 0;i<sp_idx_known.size();i++)
	{
		int sp_idx = sp_idx_known[i];

		cv::Mat H;
		NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx],H);
		homoS[sp_idx] = H;

		//DrawHomoWarp(nrdc, SP[sp_idx], H, sp_idx, img_src.cols, img_src.rows);

        validHomoS[sp_idx] = 1;

        dDisplacementThres += NRDCProcessing::ComputeDisplacementOfSuperpixel(SP, sp_idx, H);
	}

    dDisplacementThres /= (double)(sp_idx_known.size());
    std::cout << "Displacement thres: " << dDisplacementThres << ", imgWidth/8: " << img_src.cols/8 << std::endl;

    dDisplacementThres *= 2;

    t2 = omp_get_wtime();
    std::cout << "compute known homography use time: " << (t2 - t1) << std::endl;

    
    // inconsisntent superpixel
    //#pragma omp parallel for
	for(int i = 0;i<sp_idx_missing.size();i++)
	{
		size_t sp_idx = sp_idx_missing[i];
		std::cout << "missing superpixel id: " << sp_idx<<'\n'<<std::flush;

		int nearest_boundary_idx;
		std::vector<int> H_seeds;
		cv::Mat H;			


        t1 = omp_get_wtime();
        // 找到离该superpixel最近的boundary superpixel
		NRDCProcessing::ComputeNearestBoundarySpForMissingSp(graph,
			sp_chi_diff,
			hists,
			sp_cate_label,
			sp_mask,
			sp_idx,
			nearest_boundary_idx);
        t2 = omp_get_wtime();
        //std::cout << "ComputeNearestBoundarySpForMissingSp use time: " << (t2 - t1) << std::endl;

       
        t1 = omp_get_wtime();
		NRDCProcessing::ComputeSeedsForHomographyInference(graph,sp_chi_diff,sp_cate_label,
			sp_mask,
			nearest_boundary_idx,H_seeds);
        t2 = omp_get_wtime();
        //std::cout << "ComputeSeedsForHomographyInference use time: " << (t2 - t1) << std::endl;

	    t1 = omp_get_wtime();
		NRDCProcessing::ComputeHomographyForSP(nrdc,SP,H_seeds,H);

        //dDisplacementThres = 10;

        if (NRDCProcessing::CheckIfPropogatedHomographyReliable(SP, sp_idx, H, dDisplacementThres))
        {
            validHomoS[sp_idx] = 1;
        }

		homoS[sp_idx] = H;
		homoT[sp_idx] = H.clone();
        t2 = omp_get_wtime();
        //std::cout << "ComputeHomographyForSP use time: " << (t2 - t1) << std::endl;

        cv::Mat imgggg(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0));
       
        // show current sp
        for(size_t j = 0;j<SP[sp_idx].size();j++)
        {
            v2i pos = SP[sp_idx][j];
            
            int r,g,b;
            LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
            LibIV::CppHelpers::Global::setImgData(255,255,0,imgggg,pos[0],pos[1]);
        }
        
        for(size_t j = 0;j<H_seeds.size();j++)
        {
            int idx = H_seeds[j];
            for(size_t k = 0;k<SP[idx].size();k++)
            {
                v2i pos = SP[idx][k];

                int r,g,b;
                LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
                LibIV::CppHelpers::Global::setImgData(r,g,b,imgggg,pos[0],pos[1]);
            }
        }

        for(size_t j = 0;j<SP[nearest_boundary_idx].size();j++)
        {
            v2i pos = SP[nearest_boundary_idx][j];

            int r,g,b;
            LibIV::CppHelpers::Global::imgData(r,g,b,img_src,pos[0],pos[1]);
            LibIV::CppHelpers::Global::setImgData(255,255,255,imgggg,pos[0],pos[1]);
        }

        char filename[1024];
        if (NRDCTest::isRunningBackward)
        {
            sprintf_s(filename,1024,"D:/helloaaa/backward/%d.png",sp_idx);
        }else
        {
            sprintf_s(filename,1024,"D:/helloaaa/forward/%d.png",sp_idx);
        }
        cv::imwrite(filename,imgggg);
        
	}



    t1 = omp_get_wtime();
    //std::cout << "compute missing homography using time: " << (t1 - t2) << std::endl;

	std::cout<<"change SP to Eigen_SP ..."<<std::flush;
	std::vector<std::vector<Eigen::Vector2i>> Eigen_SP;

//#pragma omp parallel for
	for(int i = 0; i<SP.size();i++)
	{
		std::vector<Eigen::Vector2i> tmp;
		for(int j = 0;j<SP[i].size();j++)
		{
			v2i pos = SP[i][j];
			tmp.push_back(Eigen::Vector2i(pos[0],pos[1]));
		}
		Eigen_SP.push_back(tmp);
	}

	// we dilate the missing region to get a band buffer overlaped with known region that 
	// provides control points
	std::cout<<"band buffer ..."<<std::flush;		
	cv::Mat img_mask_known, img_mask_missing, img_mask_dilation, img_band_buffer;

	img_mask_known = cv::Mat(img_src.rows,img_src.cols,CV_8UC1,cv::Scalar(0));
	img_mask_missing = cv::Mat(img_src.rows,img_src.cols,CV_8UC1,cv::Scalar(0));
	
	for(size_t i = 0;i<sp_idx_known.size();i++)
	{
		int sp_idx = sp_idx_known[i];
		for(size_t j = 0;j<SP[sp_idx].size();j++)
		{
			v2i pos = SP[sp_idx][j];
			LibIV::CppHelpers::Global::setImgData(255,img_mask_known,pos[0],pos[1]);
		}
	}

	for(size_t i = 0;i<sp_idx_missing.size();i++)
	{
		int sp_idx = sp_idx_missing[i];
		for(size_t j = 0;j<SP[sp_idx].size();j++)
		{
			v2i pos = SP[sp_idx][j];
			
			LibIV::CppHelpers::Global::setImgData(255,img_mask_missing,pos[0],pos[1]);
		}
	}

	int dilation_size = 5;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		cv::Point(dilation_size,dilation_size));
	cv::dilate(img_mask_missing,img_mask_dilation,element);

	img_band_buffer = cv::Mat(img_src.rows,img_src.cols,CV_8UC1,cv::Scalar(0));

	for(int i = 0;i<img_src.rows;i++)
	{
		for(int j = 0;j<img_src.cols;j++)
		{
			int g1,g2;
			LibIV::CppHelpers::Global::imgData(g1,img_mask_known,j,i);
			LibIV::CppHelpers::Global::imgData(g2,img_mask_dilation,j,i);
			if(g1 == 255 && g2 == 255)
				LibIV::CppHelpers::Global::setImgData(255,img_band_buffer,j,i);
		}
	}
	
    if (!NRDCTest::isRunningBackward)
    {
        // forward
        cv::imwrite(std::string("D:/forward_mask_known" + std::to_string(mask_id) + ".png").c_str(),img_mask_known);
        cv::imwrite(std::string("D:/forward_mask_missing" + std::to_string(mask_id) + ".png").c_str(),img_mask_missing);
        cv::imwrite(std::string("D:/forward_mask_dilation.png" + std::to_string(mask_id) + ".png").c_str(),img_mask_dilation);
        cv::imwrite(std::string("D:/forward_mask_band_buffer.png"+ std::to_string(mask_id) + ".png").c_str(),img_band_buffer);
    }else
    {
        // backward
        cv::imwrite(std::string("D:/backward_mask_known" + std::to_string(mask_id) + ".png").c_str(),img_mask_known);
        cv::imwrite(std::string("D:/backward_mask_missing" + std::to_string(mask_id) + ".png").c_str(),img_mask_missing);
        cv::imwrite(std::string("D:/backward_mask_dilation.png" + std::to_string(mask_id) + ".png").c_str(),img_mask_dilation);
        cv::imwrite(std::string("D:/backward_mask_band_buffer.png"+ std::to_string(mask_id) + ".png").c_str(),img_band_buffer);
    }

	cout<<"min and max..."<<std::flush;
	int min_x,min_y,max_x,max_y;
	min_x = min_y = 100000;
	max_x = max_y =-100000;

	for(int i = 0;i<img_src.rows;i++)
	{
		for(int j = 0;j<img_src.cols;j++)
		{
			int g1,g2;
			LibIV::CppHelpers::Global::imgData(g1,img_mask_missing,j,i);
			LibIV::CppHelpers::Global::imgData(g2,img_band_buffer,j,i);
			if(g1 == 255 || g2 == 255)
			{
				if(j < min_x) min_x = j;
				if(j > max_x) max_x = j;
				if(i < min_y) min_y = i;
				if(i > max_y) max_y = i;
			}
			
		}
	}

	cout<<"build triangle mesh ..."<<std::flush;
	std::vector<Triangle>			all_triangles;
	std::vector<Vertex>				all_vertices;
	std::vector<Vertex>				all_deformed_vertices;
	std::vector<Eigen::Vector2d>	control_points;
	std::vector<Eigen::Vector2d>	control_to_points;
	std::vector<FeaturePoint>		tmp_points;

	all_triangles.clear();
	all_vertices.clear();
	all_deformed_vertices.clear();
	control_points.clear();
	control_to_points.clear();
	tmp_points.clear();

	cout<<"set control points ..."<<std::flush;
	for(size_t i = 0;i<sp_idx_known.size();i++)
	{
		int sp_idx = sp_idx_known[i];
		
		std::vector<v2d> corre;
		NRDCProcessing::HomographyTransform(SP[sp_idx],homoS[sp_idx],corre);

		for(size_t j = 0;j<corre.size();j++)
		{
			v2i pos  = SP[sp_idx][j];
			v2d posc = corre[j];

			int gray;
			LibIV::CppHelpers::Global::imgData(gray,img_band_buffer,pos[0],pos[1]);
			if(gray != 255) // the pixels that undergo conherent homography 
			{
				src_dst_corre.at(pos[1],pos[0]) = _v3d_(mask_id,posc[0],posc[1]);
                
			}
			else
			{
				if(j % 10 == 0) // sample a control point every ten pixels for time saving
				{
					control_points.push_back(Eigen::Vector2d(pos[0],pos[1]));
					control_to_points.push_back(Eigen::Vector2d(posc[0],posc[1]));
				}
			}
		}
	}

    if (sp_idx_missing.size() == 0)
    {
        return;
    }

    Geometry2D::buildTriangleMesh(min_x,max_x,min_y,max_y,all_vertices,all_triangles);
    WarpRender wr;

	cout<<"perform warping ..."<<std::flush;

    
	PCWarpSolverAdaptive pcws;
	pcws.performWarping(img_src.cols,
		img_src.rows,
		all_vertices,
		all_deformed_vertices,
		all_triangles,
		control_points,
		control_to_points,
		Eigen_SP,
		homoS,
		sp_idx_missing,
		homoT,
        validHomoS,
		1,
		1,
		1e5,
		1e5);

    cv::Mat testImage(img_src.rows, img_src.cols, CV_8UC3, cv::Scalar(0));
    wr.drawMesh(all_deformed_vertices, all_triangles, testImage);


    if (NRDCTest::isRunningBackward)
    {
        cv::imwrite(std::string("D:/backward_mesh" + std::to_string(mask_id) + ".png").c_str(),testImage);
    }else
    {
        cv::imwrite(std::string("D:/forward_mesh" + std::to_string(mask_id) + ".png").c_str(),testImage);
    }

//    cv::imwrite("D:/mesh.bmp", testImage);

	cout<<"missing pixels undergo warping ..."<<endl;
	for(size_t i = 0;i<all_triangles.size();i++)
	{
		//cout<<i<<endl;
		int ver_idx[3];
		ver_idx[0] = (int)(all_triangles[i].nVertices[0]);
		ver_idx[1] = (int)(all_triangles[i].nVertices[1]);
		ver_idx[2] = (int)(all_triangles[i].nVertices[2]);

		std::vector<Eigen::Vector2d> eigen_triangle;
		std::vector<Vertex>   deformed_ver;
		double tri_min_x,tri_min_y,tri_max_x,tri_max_y;
		tri_min_x = tri_min_y = 1e10;
		tri_max_x = tri_max_y = -1e10;	

		for(int j = 0;j<3;j++)
		{
			Vertex ver = all_vertices[ver_idx[j]];
			
			if(ver.vPosition.x() < tri_min_x) tri_min_x = ver.vPosition.x();
			if(ver.vPosition.x() > tri_max_x) tri_max_x = ver.vPosition.x();
			if(ver.vPosition.y() < tri_min_y) tri_min_y = ver.vPosition.y();
			if(ver.vPosition.y() > tri_max_y) tri_max_y = ver.vPosition.y();

			eigen_triangle.push_back(Eigen::Vector2d(ver.vPosition.x(),ver.vPosition.y()));
			deformed_ver.push_back(all_deformed_vertices[ver_idx[j]]);
		}

		for(int ii = (int)tri_min_y;ii<=(int)tri_max_y;ii++)
		{
			for(int jj = (int)tri_min_x;jj<=(int)tri_max_x;jj++)
			{
				int gray;
				LibIV::CppHelpers::Global::imgData(gray,img_mask_missing,jj,ii);
				if(gray != 255)
					continue;
				
				std::vector<double> weights;
				Eigen::Vector2d cur_pos(jj,ii);
				Geometry2D::BarycentricCoord(eigen_triangle,cur_pos,weights);
				
				bool is_in_triangle = true;
				for(int j = 0;j<3;j++)
					if(weights[j] < 0)	
					{
						is_in_triangle = false;
						break;
					}

				if(!is_in_triangle)
					continue;

				v2d posc = _v2d_(0,0);
				for(int j = 0;j<3;j++)
				{
					posc[0] += weights[j] * deformed_ver[j].vPosition.x();
					posc[1] += weights[j] * deformed_ver[j].vPosition.y();
				}

				// pixels that undergo warping
#ifndef ONLYRELIABLE
				src_dst_corre.at(ii,jj) = _v3d_(mask_id,posc[0],posc[1]);              
#endif               
			}
		}
	}

	cout<<"band pixels undergo blending ..."<<std::flush;

	for(int i = 0;i<img_src.rows;i++)
	{
		for(int j = 0;j<img_src.cols;j++)
		{
			int gray;
			LibIV::CppHelpers::Global::imgData(gray,img_band_buffer,j,i);
			if(gray != 255) 
				continue;
			
			int radius = 0;
			std::vector<v2i> missing_coord;
			std::vector<v2i> known_coord;
			bool is_missing_found = false;
			bool is_known_found = false;
			
			while(!is_missing_found)
			{
				int or = radius;
				radius = or + 5;

				for(int n = i - radius;n<=i+radius;n++)
				{
					for(int m = j - radius;m<=j + radius;m++)
					{
						if(n >= i - or && n <= i + or &&
							m >= j - or && m <= j + or)
							continue;
						if(n >= 0 && n < img_src.rows &&
							m >= 0 && m < img_src.cols)
						{
							int g2;
							LibIV::CppHelpers::Global::imgData(g2,img_mask_missing,m,n);

							if(g2 == 255)
								missing_coord.push_back(_v2i_(m,n));
						}
					}
				}

				if(!missing_coord.empty()) is_missing_found = true;
			}

			radius = 0;
			while(!is_known_found)
			{
				int or = radius;
				radius = or + 5;

				for(int n = i - radius;n<=i+radius;n++)
				{
					for(int m = j - radius;m<=j + radius;m++)
					{
						if(n >= i - or && n <= i + or &&
							m >= j - or && m <= j + or)
							continue;
						if(n >= 0 && n < img_src.rows &&
							m >= 0 && m < img_src.cols)
						{
							int g1,g3;
							LibIV::CppHelpers::Global::imgData(g1,img_mask_known,m,n);
							LibIV::CppHelpers::Global::imgData(g3,img_band_buffer,m,n);

							if(g1 == 255 && g3 != 255)
								known_coord.push_back(_v2i_(m,n));
						}
					}
				}

				if(!known_coord.empty()) is_known_found = true;
			}

			std::vector<double> missing_dist, known_dist;

			for(size_t k = 0;k<missing_coord.size();k++)
			{
				double xxx = missing_coord[k][0] - j;
				double yyy = missing_coord[k][1] - i;
				missing_dist.push_back(sqrt(xxx * xxx + yyy * yyy));
			}

			for(size_t k = 0;k<known_coord.size();k++)
			{
				double xxx = known_coord[k][0] - j;
				double yyy = known_coord[k][1] - i;
				known_dist.push_back(sqrt(xxx * xxx + yyy * yyy));
			}

			int missing_min, known_min;
			double missing_dist_min = 1e10, known_dist_min = 1e10;
			for(size_t k = 0;k<missing_dist.size();k++)
			{
				if(missing_dist[k] < missing_dist_min) 
				{
					missing_dist_min = missing_dist[k];
					missing_min = (int)k;
				}
			}
			for(size_t k = 0;k<known_dist.size();k++)
			{
				if(known_dist[k] < known_dist_min) 
				{
					known_dist_min = known_dist[k];
					known_min = (int)k;
				}
			}

			if(missing_coord.empty() || known_coord.empty())
			{
				std::cout<<" ===========================error2028"<<std::endl;
				continue;
			}

			v3d missing_v3d = src_dst_corre.at(missing_coord[missing_min][1],missing_coord[missing_min][0]);
			v3d known_v3d   = src_dst_corre.at(known_coord[known_min][1],known_coord[known_min][0]);
			
			v2d missing_motion = _v2d_(missing_v3d[1] - missing_coord[missing_min][0],
				missing_v3d[2] - missing_coord[missing_min][1]);
			v2d known_motion = _v2d_(known_v3d[1] - known_coord[known_min][0],
				known_v3d[2] - known_coord[known_min][1]);
			
			if(missing_v3d[0] == 0)
			{
				//std::cout<<" ===========================error3008"<<std::endl;
				continue;
			}

			if(known_v3d[0] == 0)
			{
				std::cout<<" ===========================error4008"<<std::endl;
				continue;
			}
			
			double dAB = known_dist_min + missing_dist_min;
			double xxxx = missing_motion[0] * known_dist_min / dAB + 
				known_motion[0] * missing_dist_min / dAB;
			double yyyy = missing_motion[1] * known_dist_min / dAB + 
				known_motion[1] * missing_dist_min / dAB;
			
			// pixels that undergo blending
#ifndef ONLYRELIABLE
            src_dst_corre.at(i,j) = _v3d_(mask_id,xxxx + j,yyyy + i);
#endif
		}
	}

	cout<<"done"<<endl;
}

inline float very_fast_exp(float x) {
	return 1
		-x*(0.9999999995f
		-x*(0.4999999206f

		-x*(0.1666653019f
		-x*(0.0416573475f
		-x*(0.0083013598f

		-x*(0.0013298820f
		-x*(0.0001413161f)))))));
}

inline float fast_exp(float x) {
	bool lessZero = true;
	if (x < 0) {
		lessZero = false;
		x = -x;
	}

	// This dirty little trick only works because of the normalization and the fact that one element in the normalization is 1
	if (x > 20)
		return 0;
	int mult = 0;

	while (x > 0.69*2*2*2) {
		mult+=3;
		x /= 8.0f;
	}

	while (x > 0.69*2*2) {
		mult+=2;
		x /= 4.0f;
	}
	while (x > 0.69) {
		mult++;
		x /= 2.0f;
	}

	x = very_fast_exp(x);
	while (mult) {
		mult--;
		x = x*x;
	}

	if (lessZero) {
		return 1 / x;

	} else {
		return x;
	}
}


void NRDCProcessing::InverseMapping(const cv::Mat & img_src, cv::Mat & img_show, cv::Mat & img_hole,const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, const std::vector<v2i> & coord, const std::vector<v2d> & coordc)
{
	cout << "inverse mapping ..." << std::endl << std::flush;

    if(coord.empty()) return;

	// simple rendering
	//for(size_t i = 0;i<coordc.size();i++)
 //   {
 //       v2d posc = coordc[i];
 //       v2i pos  =  coord[i];

 //      

 //       int r,g,b;
 //       LibIV::CppHelpers::Global::imgData(r, g, b, img_src, pos[0], pos[1]);

	//	if(posc[0] >= 0 && posc[0] < img_src.cols && posc[1] >= 0 && posc[1] < img_src.rows)
	//	{
	//		LibIV::CppHelpers::Global::setImgData(r, g, b, img_show, posc[0],posc[1]);
	//	}      
 //   }

	//return;

	// end simple rendering



	
    //LibIV::Memory::Array::FastArray2D<int> isOccupied;
    //isOccupied.set(img_src.cols, img_src.rows);
    //isOccupied.fill(0);

    //std::vector<Eigen::Vector2d> vPoints;
    //vPoints.clear();
    //std::vector<Eigen::Vector2i> vSegments;
    //vSegments.clear();

    //int gridSize = 20;

    //for (int i = 0; i < coordc.size(); i++)
    //{

    //    int gridx = (int)coordc[i][0] / gridSize;
    //    int gridy = (int)coordc[i][1] / gridSize;

    //    if (gridx >= 0 && gridx < img_src.cols && gridy >= 0 && gridy < img_src.rows
    //        && !isOccupied.at(gridy, gridx))
    //    {
    //        isOccupied.at(gridy, gridx) = 1;
    //        vPoints.push_back(Eigen::Vector2d(coordc[i][0], coordc[i][1]));
    //    }
    //}

    //triangulateio *in = Geometry2D::InputToTriangulateio(vPoints, vSegments);
    //triangulateio *out = Geometry2D::ComputeMeshByTriangle(in, true);

    //std::vector<Vertex> vVertices;
    //std::vector<Triangle> vTriangles;
    //vVertices.clear();
    //vTriangles.clear();

    //// Initialize mesh triangles and the position of the vertices
    //Geometry2D::TriangulateioToOutput(out, vVertices, vTriangles);
    //Geometry2D::FreeTriangulateio(&in, true);
    //Geometry2D::FreeTriangulateio(&out, false);

    //cv::Mat output(img_src.rows,img_src.cols, CV_8UC3, cv::Scalar(0,0,0));



    //for (int i = 0; i < coordc.size(); i++)
    //{
    //    //cv::circle(output, cv::Point(coordc[i][0], coordc[i][1]), 2, cv::Scalar(255, 255, 255));

    //    if (coordc[i][0] < 200 && coordc[i][1] < 200)
    //    {
    //        output.at<cv::Vec3b>((int)(coordc[i][1]), (int)(coordc[i][0])) = cv::Vec3b(255, 255, 255);
    //    }
    //}

    
  /*  WarpRender wr;
    wr.drawMesh(vVertices, vTriangles, output);

    cv::imwrite("D:/testTriangle11111111.bmp", output);*/

    LibIV::Memory::Array::FastArray2D<v6d> accum;
    accum.set(img_src.cols,img_src.rows);
    accum.fill(_v6d_(0,0,0,0,0,0));

    int width = src_dst_corre.cols();
    int height = src_dst_corre.rows();

    double influ_size = 3;
    double influ_var  = 1.5;

    for(size_t i = 0;i<coordc.size();i++)
    {
        v2d posc = coordc[i];
        v2i pos  =  coord[i];

        v2d motion = _v2d_(pos[0] - posc[0],pos[1] - posc[1]);

        int r,g,b;
        LibIV::CppHelpers::Global::imgData(r, g, b, img_src, pos[0], pos[1]);


        for(int ii = (int)(posc[1] - influ_size);ii<=(int)(posc[1] + influ_size);ii++)
        {
            for(int jj = (int)(posc[0] - influ_size);jj<=(int)(posc[0] + influ_size);jj++)
            {
                if(ii>=0 && ii <img_src.rows &&
                    jj>=0 && jj < img_src.cols)
                {
                    double xx = posc[0] - jj;
                    double yy = posc[1] - ii;



                    double ww = exp(-0.5 * (xx*xx+yy*yy) / (influ_var * influ_var));   //exp
                   // double ww = 1;


                    v6d tmp = accum.at(ii,jj);
                    tmp[0] += ww * motion[0];
                    tmp[1] += ww * motion[1];
                    tmp[2] += ww * r;
                    tmp[3] += ww * g;
                    tmp[4] += ww * b;
                    tmp[5] += ww;
                    accum.at(ii,jj) = tmp;
                }
            }
        }
    }


    for(int i = 0;i<img_src.rows;i++)
    {
        for(int j = 0;j<img_src.cols;j++)
        {
            v6d inv_motion = accum.at(i,j);
            if(inv_motion[5] == 0  )
            {
            } 
            else
            {
                double x = inv_motion[0] / inv_motion[5];
                double y = inv_motion[1] / inv_motion[5];

                if(x+j >= 0 && y+i >= 0 && x+j < width - 1 && y+i < height - 1)
                {
                    int xx = (int)(x+j);
                    int yy = (int)(y+i);

                    if(src_dst_corre.at(yy,xx)[0] >= 0 && 
                        src_dst_corre.at(yy,xx+1)[0] >= 0 &&
                        src_dst_corre.at(yy+1,xx)[0] >= 0 &&
                        src_dst_corre.at(yy+1,xx+1)[0] >= 0)
                    {
                        int r,g,b;

                        NRDCProcessing::ImageGet2D(r,g,b,img_src,x + j, y + i);

                        LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,j,i);
                        LibIV::CppHelpers::Global::setImgData(255, 255, 255, img_hole, j, i);
                    }
                }
            }
        }
    }
}

void NRDCProcessing::InverseMapping(const cv::Mat & img_src, cv::Mat & img_show, cv::Mat & img_hole, const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, int label_num)
{
	std::vector<v2i> coord;
	std::vector<v2d> coordc;
    
    LibIV::Memory::Array::FastArray2D<v3d> object_mask;
    object_mask.set(img_src.cols,img_src.rows);
    object_mask.fill(_v3d_(-1,-1,-1));

	for(int i = 1;i<=label_num;i++)
	{
		coord.clear();
		coordc.clear();

		for(int j = 0;j<img_src.rows;j++)
		{
			for(int k = 0;k<img_src.cols;k++)
			{
				v3d tmp = src_dst_corre.at(j,k);
				if(tmp[0] == i)
				{
					coord.push_back(_v2i_(k,j));
					coordc.push_back(_v2d_(tmp[1],tmp[2]));
                    object_mask.at(j,k) = _v3d_(i,0,0);
				}
			}
		}
		NRDCProcessing::InverseMapping(img_src,img_show, img_hole,object_mask, coord,coordc);
	}
}

void NRDCProcessing::ViewInterpolation(std::string data_filepath,const cv::Mat & img_src, const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, int label_num)
{
	cout<<"view interpolation ..."<<std::flush;
	for(int i = 0;i<100;i++)
	{
		cout<<i<<endl;
		double ss = (double)i/100.0;

		LibIV::Memory::Array::FastArray2D<v3d> tmp;
		tmp.set(img_src.cols,img_src.rows);
		tmp.fill(_v3d_(0,0,0));

		for(int j = 0;j<img_src.rows;j++)
		{
			for(int k = 0;k<img_src.cols;k++)
			{
				v2i pos = _v2i_(k,j);
				v3d cc = src_dst_corre.at(j,k);
				v2d posc = _v2d_(cc[1],cc[2]);

				double x = posc[0] - pos[0];
				double y = posc[1] - pos[1];

				x = ss * x + pos[0];
				y = ss * y + pos[1];

				tmp.at(j,k) = _v3d_(cc[0],x,y);
			}
		}
		cv::Mat img_show(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::Mat img_hole(img_src.rows,img_src.cols,CV_8UC3,cv::Scalar(0,0,0));
		NRDCProcessing::InverseMapping(img_src,img_show,img_hole, tmp, label_num);

		char filename[1024];
		sprintf_s(filename,1024,"%s/%d.png",(data_filepath+"/viewinterp").c_str(),i);
		cv::imwrite(filename,img_show);
	}
}



void NRDCProcessing::ComputeSrcDstCompleteCorre(std::string data_filepath, const std::vector<int> & segmentation_label, cv::Mat & img_src, LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre)
{
	img_src = cv::imread((data_filepath + "/a.bmp").c_str());

	int width = img_src.cols;
	int height = img_src.rows;

    // (maskid, corrx, corry)
    // maskid == -1 indicate this pixel belongs to incoherence region
    // maskid == 0, this pixel belongs to coherence region, but located on the boundary
    // maskid == 1, this pixel belongs to coherence region, not located on the boundary
	src_dst_corre.set(width,height);
	src_dst_corre.fill(_v3d_(0,0,0));

	std::vector<std::vector<v2i>> SP;
	NRDC nrdc(height,width);
	SpGraph * graph;

	NRDCProcessing::ReadSP((data_filepath + "/SP.txt").c_str(),SP);
	NRDCProcessing::ReadNRDC(nrdc,(data_filepath + "/NRDC.txt").c_str());
	graph = NRDCProcessing::MakeSpGraph(SP,width,height);

	Histograms hists;
	SpChiDiff sp_chi_diff;

	NRDCProcessing::ComputeHistogramForSPs(img_src,SP,hists);
	NRDCProcessing::ChiSquareDistOfEdge(graph,hists,sp_chi_diff);


    // coherence区域1，coherence边界区域0,incoherence区域-1
	std::vector<int> sp_cate_label;
	NRDCProcessing::DivideSpCategory(graph,SP,nrdc,sp_cate_label);

	// ====================================================
	// processing begins at here

	cout<<"TestLocalWarping ... Processing begins at here ... "<<std::flush;
	
	int label_num = -1000;
	for(size_t i = 0;i<segmentation_label.size();i++)
	{
		if(segmentation_label[i] > label_num)
			label_num = segmentation_label[i];
	}

	std::vector<std::vector<int>> segmentation(label_num+1);
	for(int i = 0; i<= label_num;i++)
		segmentation[i].resize(graph->size(),0);

	for(size_t i = 0;i<segmentation_label.size();i++)
		segmentation[segmentation_label[i]][i] = 1;

    // 分层warp,最远的最先warp，最近的最后warp
	for(int i = 1;i <= label_num;i++)
	{
		NRDCProcessing::LocalWarping2(
			img_src,
			SP,
			nrdc,
			graph,
			hists,
			sp_chi_diff,
			segmentation[i],
			sp_cate_label,
			src_dst_corre,
			i
			);
	}

	// ====================================================
	NRDCProcessing::DeleteSpGraph(&graph);
	SP.clear();
	nrdc.erase();
	hists.clear();
	sp_chi_diff.clear();
	sp_cate_label.clear();
}

void NRDCProcessing::ViewInterpolation(std::string store_path, int label_num,
									   const cv::Mat & img_forward, const cv::Mat & img_backward, 
									   const LibIV::Memory::Array::FastArray2D<v3d> & corre_forward, 
									   const LibIV::Memory::Array::FastArray2D<v3d> & corre_backward)
{
	cout<<"view interpolation ..."<<std::flush;

    #pragma omp for
	for(int i = 0;i<100;i++)
	{
		cout<<i<<endl;
		double ss = (double)i/100.0;

		LibIV::Memory::Array::FastArray2D<v3d> tmp;
		tmp.set(img_forward.cols,img_forward.rows);
		tmp.fill(_v3d_(0,0,0));

		for(int j = 0;j<img_forward.rows;j++)
		{
			for(int k = 0;k<img_forward.cols;k++)
			{
				v2i pos = _v2i_(k,j);
				v3d cc = corre_forward.at(j,k);
				v2d posc = _v2d_(cc[1],cc[2]);

				double x = posc[0] - pos[0];
				double y = posc[1] - pos[1];

				x = ss * x + pos[0];
				y = ss * y + pos[1];

				tmp.at(j,k) = _v3d_(cc[0],x,y);
			}
		}
		cv::Mat img_show_forward(img_forward.rows,img_forward.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::Mat img_hole_forward(img_forward.rows,img_forward.cols,CV_8UC3,cv::Scalar(0,0,0));
		NRDCProcessing::InverseMapping(img_forward,img_show_forward,img_hole_forward,tmp,label_num);

		LibIV::Memory::Array::FastArray2D<v3d> tmp_backward;
		tmp_backward.set(img_backward.cols,img_backward.rows);
		tmp_backward.fill(_v3d_(0,0,0));

		for(int j = 0;j<img_backward.rows;j++)
		{
			for(int k = 0;k<img_backward.cols;k++)
			{
				v2i pos = _v2i_(k,j);
				v3d cc = corre_backward.at(j,k);
				v2d posc = _v2d_(cc[1],cc[2]);

				double x = posc[0] - pos[0];
				double y = posc[1] - pos[1];

				x = (1-ss) * x + pos[0];
				y = (1-ss) * y + pos[1];

				tmp_backward.at(j,k) = _v3d_(cc[0],x,y);
			}
		}
		cv::Mat img_show_backward(img_backward.rows,img_backward.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::Mat img_hole_backward(img_backward.rows,img_backward.cols,CV_8UC3,cv::Scalar(0,0,0));
		NRDCProcessing::InverseMapping(img_backward,img_show_backward,img_hole_backward,tmp_backward,label_num);

		cv::Mat img_show(img_forward.rows,img_forward.cols,CV_8UC3,cv::Scalar(0,0,0));

		for(int j = 0;j<img_forward.rows;j++)
		{
			for(int k = 0;k<img_forward.cols;k++)
			{
				int r1,g1,b1,r2,g2,b2;
				LibIV::CppHelpers::Global::imgData(r1,g1,b1,img_show_forward,k,j);
				LibIV::CppHelpers::Global::imgData(r2,g2,b2,img_show_backward,k,j);

				int r = (int)(r1 * (1-ss) + r2 * ss);
				int g = (int)(g1 * (1-ss) + g2 * ss);
				int b = (int)(b1 * (1-ss) + b2 * ss);
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,k,j);
			}
		}

		char filename[1024];
		sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp/1").c_str(),i);
		cv::imwrite(filename,img_show_forward);
		sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp/2").c_str(),i);
        cv::imwrite(filename,img_show_backward);
		//sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp").c_str(),i);
		//cv::imwrite(filename,img_show);

        sprintf_s(filename,1024,"%s/%d.hole.png",(store_path+"/viewinterp/1").c_str(),i);
        cv::imwrite(filename,img_hole_forward);
        sprintf_s(filename,1024,"%s/%d.hole.png",(store_path+"/viewinterp/2").c_str(),i);
        cv::imwrite(filename,img_hole_backward);
	}
}

void NRDCProcessing::ViewInterpolation (std::string store_path, 
                                             int label_num, 
                                             const cv::Mat & img_forward, 
                                             const LibIV::Memory::Array::FastArray2D<v3d> & corre_forward,
                                             bool isForward)
{
    cout<<"view interpolation ..."<<std::flush;
	char filename[1024];

	// begin simple rendering

	//IplImage * img_show = cvCreateImage(cvSize(img_forward.cols,img_forward.rows),8,3);
	//

	//for(int k = 0;k<100;k++)
	//{
	//	cvZero(img_show);
	//	for(int i = 0;i<corre_forward.rows();i++)
	//	{
	//		for(int j = 0;j<corre_forward.cols();j++)
	//		{
	//			int xx = corre_forward.at(i,j)[1];
	//			int yy = corre_forward.at(i,j)[2];

	//			//if(xx < 0 || xx >= corre_forward.cols() || yy < 0 || yy >= corre_forward.rows() || corre_forward.at(i,j)[0] <= 0)
	//			//	continue;

	//			int x,y;
	//			x = (xx-j)/100.0 * k + j;
	//			y = (yy-i)/100.0 * k + i;

	//			if(x >= 0 && x < img_forward.cols && y>=0 && y < img_forward.rows)// && corre_forward.at(i,j)[0] > 0)
	//			{
	//				int r,g,b;
	//				LibIV::CppHelpers::Global::imgData(r,g,b,img_forward,j,i);
	//				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,x,y);
	//			}
	//		}
	//	}
	//	sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp/1").c_str(),k);
	//	cvSaveImage(filename,img_show);
	//}
	//cvReleaseImage(&img_show);

	//return;
	
	
	// end simple rendering

    //#pragma omp for
	int ii=1;
    for(int i = 0;i<100;i+=ii)
    {
		//if(i > 4) ii=8;
        cout<<i<<endl;
        double ss = (double)i/100.0;

        LibIV::Memory::Array::FastArray2D<v3d> tmp;
        tmp.set(img_forward.cols,img_forward.rows);
        tmp.fill(_v3d_(0,0,0));

        for(int j = 0;j<img_forward.rows;j++)
        {
            for(int k = 0;k<img_forward.cols;k++)
            {
                v2i pos = _v2i_(k,j);
                v3d cc = corre_forward.at(j,k);
                v2d posc = _v2d_(cc[1],cc[2]); // cc[0] is the mask id

                double x = posc[0] - pos[0];
                double y = posc[1] - pos[1];


#ifdef __FORWARD
                x = ss * x + pos[0];
                y = ss * y + pos[1];
#endif // __FORWARD

#ifdef __BACKWARD
                x = (1 - ss) * x + pos[0];
                y = (1 - ss) * y + pos[1];
#endif // __BACKWARD

               

                tmp.at(j,k) = _v3d_(cc[0],x,y);
            }
        }


        cv::Mat img_show_forward(img_forward.rows,img_forward.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::Mat img_hole_forward(img_forward.rows,img_forward.cols,CV_8UC3,cv::Scalar(0,0,0));
        NRDCProcessing::InverseMapping(img_forward,img_show_forward,img_hole_forward,tmp,label_num);

        if (isForward)
        {
            sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp/1").c_str(),i);
            cv::imwrite(filename,img_show_forward);
            sprintf_s(filename,1024,"%s/%d.hole.png",(store_path+"/viewinterp/1").c_str(),i);
            cv::imwrite(filename,img_hole_forward);
        }else
        {
            sprintf_s(filename,1024,"%s/%d.png",(store_path+"/viewinterp/2").c_str(),i);
            cv::imwrite(filename,img_show_forward);
            sprintf_s(filename,1024,"%s/%d.hole.png",(store_path+"/viewinterp/2").c_str(),i);
            cv::imwrite(filename,img_hole_forward);
        }
    } 
}

void NRDCProcessing::SaveIntermediateVariables(
        std::string fileName,
        std::vector<cv::Mat> &homoS,
        std::vector<int> &validHomoS)
{
	ofstream outfile(fileName,ios::binary);

	union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

    bar.d = (double)homoS.size();
    outfile.write(bar.c,sizeof(double));

    for (int k = 0; k < homoS.size(); k++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                bar.d = homoS[k].at<double>(i,j);
                outfile.write(bar.c,sizeof(double));
            }
        }

        bar.d = validHomoS[k];
        outfile.write(bar.c,sizeof(double));
    }

	outfile.close();
}


void NRDCProcessing::LoadIntermediateVariables(
        std::string fileName,
        std::vector<cv::Mat> & homoS,
        std::vector<int> & validHomoS)
{
    union foo
    { 
		char c[sizeof(double)]; 
		double d;
    }bar;

	ifstream infile(fileName,ios::binary);
	
	infile.seekg(0,ios::end);
    long long int ss = infile.tellg();
    char * buf = new char[(unsigned int)ss];
    infile.seekg(0,ios::beg);
    infile.read(buf,ss);
    char * p = buf;

	memcpy(bar.c,p,sizeof(double));
	int numOfSPs = (int)(bar.d);
	p += sizeof(double);

    homoS.clear();
    validHomoS.clear();

    homoS.resize(numOfSPs, cv::Mat::zeros(3,3,CV_64F));
    validHomoS.resize(numOfSPs, 0);

    for (int k = 0; k < homoS.size(); k++)
    {
        homoS[k] = cv::Mat(3,3,CV_64F);
        for (int i = 0; i < 3; i++)
        {
            
            for (int j = 0; j < 3; j++)
            {
                memcpy(bar.c,p,sizeof(double));
	            p += sizeof(double);
                

                homoS[k].at<double>(i,j) = bar.d;
                
            }
        }
        
        

        memcpy(bar.c,p,sizeof(double));
        p += sizeof(double);
        validHomoS[k] = (int)(bar.d);
    }
    

    //std::cout << homoS[0] << std::endl;

	delete [] buf;
}

