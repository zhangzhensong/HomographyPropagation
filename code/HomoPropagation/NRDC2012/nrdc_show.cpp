#include "nrdc_show.h"
#include "nrdc_processing.h"
#include <opencv2\calib3d\calib3d.hpp>


void NRDCShow::ShowSP(IplImage * img,const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx, const char * filename)
{
	if(sp_idx.empty()) return;

	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	NRDCShow::ShowSP(img,img_show,SP,sp_idx[0],0.5);

	for(size_t j = 1;j<sp_idx.size();j++)
		NRDCShow::ShowSP(img,img_show,SP,sp_idx[j],1);

	cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);
}

void NRDCShow::ShowSP(IplImage * img, const std::vector<std::vector<v2i>> & SP, int sp_idx, const char * filename)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	NRDCShow::ShowSP(img,img_show,SP,sp_idx,1);

	cvSaveImage(filename,img_show);
	cvZero(img_show);
}

void NRDCShow::ShowSP(IplImage * img, const std::vector<v2i> & sp, const char * filename)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];
		CvScalar cs = cvGet2D(img,pos[1],pos[0]);
		cvSet2D(img_show,pos[1],pos[0],cs);
	}

	cvSaveImage(filename,img_show);
	cvZero(img_show);
}

void NRDCShow::ShowSP(IplImage * img, IplImage * img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, double alpha)
{
	int r,g,b;
	r = g = b = 255;

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		CvScalar cs = cvGet2D(img,pos[1],pos[0]);

		cs.val[0] = cs.val[0] * alpha + b * (1-alpha);
		cs.val[1] = cs.val[1] * alpha + g * (1-alpha);
		cs.val[2] = cs.val[2] * alpha + r * (1-alpha);

		cvSet2D(img_show,pos[1],pos[0],cs);
	}
}
void NRDCShow::ShowSPc(IplImage * img, IplImage * img_show, const std::vector<v2i> & sp, const std::vector<v2d> & spc)
{
	IplImage * sp_img = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(sp_img);
	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];

		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img,pos[0],pos[1]);
		LibIV::CppHelpers::Global::setImgData(r,g,b,sp_img,pos[0],pos[1]);
	}

	//==============================================================

	std::vector<cv::Point2d> sp_mat;
	std::vector<cv::Point2d> spc_mat;

	for(size_t i = 0;i<sp.size();i++)
	{
		sp_mat.push_back(cv::Point2d(sp[i][0],sp[i][1]));
		spc_mat.push_back(cv::Point2d(spc[i][0],spc[i][1]));
	}

	cv::Mat iH = cv::findHomography(cv::Mat(spc_mat),cv::Mat(sp_mat),cv::RANSAC);

	double minx, miny, maxx,maxy;
	minx = miny = 1e10;
	maxx = maxy = -1e10;

	for(size_t i = 0;i<spc.size();i++)
	{
		v2d pos = spc[i];
		
		if(pos[0] < minx) minx = pos[0];
		if(pos[0] > maxx) maxx = pos[0];
		if(pos[1] < miny) miny = pos[1];
		if(pos[1] > maxy) maxy = pos[1];
	}

	std::vector<cv::Point2d> square;
	std::vector<cv::Point2d> ireg;

	for(int i = (int)miny; i<(int)maxy;i++)
		for(int j = (int)minx; j<(int)maxx;j++)
			square.push_back(cv::Point2d(j,i));

	ireg.resize(square.size());

	cv::perspectiveTransform(cv::Mat(square),cv::Mat(ireg),iH);
	
	for(size_t i = 0;i<square.size();i++)
	{
		int x = (int)(square[i].x);
		int y = (int)(square[i].y);

		/*int px = (int)(ireg[i].x);
		int py = (int)(ireg[i].y);

		if(x >=0 && y >= 0 && px >=0 && py >=0 
			&& x < img->width && y < img->height 
			&& px < img->width && py < img->height)
		{
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,sp_img,px,py);
			if(r != 0 || g != 0 || b != 0)
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,x,y);
		}*/

		double px = ireg[i].x;
		double py = ireg[i].y;

		if(x >=0 && y >= 0 && px >=0 && py >=0 
			&& x < img->width && y < img->height 
			&& px < img->width - 1 && py < img->height - 1)
		{
			double r,g,b;
			NRDCProcessing::BilinearInterpImage(px,py,r,g,b,sp_img);
			if(r != 0 || g != 0 || b != 0)
				LibIV::CppHelpers::Global::setImgData((int)r,(int)g,(int)b,img_show,x,y);
		}
	}

	//==============================================================
	
	cvReleaseImage(&sp_img);
}

void NRDCShow::ShowSPc(IplImage * img, IplImage * img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H)
{
	// complex mapping:
	// the whole dst image (ppix) is inversely mapped to the source image (mpix, and sp_img)
	// fill the dst image if the corresponding source pixel has value

	IplImage * sp_img = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(sp_img);
	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];

		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img,pos[0],pos[1]);
		LibIV::CppHelpers::Global::setImgData(r,g,b,sp_img,pos[0],pos[1]);
	}

	std::vector<cv::Point2d> ppix;
	std::vector<cv::Point2d> mpix;
	
	cv::Mat iH = H.inv();

	for(int i = 0;i<img->height;i++)
		for(int j = 0;j<img->width;j++)
			ppix.push_back(cv::Point2d(j,i));

	mpix.resize(ppix.size());

	cv::perspectiveTransform(cv::Mat(ppix),cv::Mat(mpix),iH);

	for(size_t i = 0;i<mpix.size();i++)
	{
		int px = (int)(ppix[i].x);
		int py = (int)(ppix[i].y);
		
		double mx = mpix[i].x;
		double my = mpix[i].y;
		
		if(mx >= 0 && my >=0
			&& mx < img->width 
			&& my < img->height)
		{
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,sp_img,(int)mx,(int)my);
			if(r != 0 || g != 0 || b != 0)
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,px,py);
		}
	}

	cvZero(sp_img);

	/*// simple mapping
	std::vector<v2d> spc;
	NRDCShow::HomographyTransform(SP[sp_idx],H,spc);

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		v2d posc = spc[i];

		if(posc[0] >= 0 && posc[0] < img->width &&
			posc[1] >= 0 && posc[1] < img->height)
			cvSet2D(img_show,(int)(posc[1]),(int)(posc[0]),cvGet2D(img,pos[1],pos[0]));
	}*/
}

void NRDCShow::ShowSPc(cv::Mat & img, cv::Mat & img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H)
{
	// complex mapping:
	// the whole dst image (ppix) is inversely mapped to the source image (mpix, and sp_img)
	// fill the dst image if the corresponding source pixel has value

	cv::Mat sp_img(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];

		int r,g,b;
		LibIV::CppHelpers::Global::imgData(r,g,b,img,pos[0],pos[1]);
		LibIV::CppHelpers::Global::setImgData(r,g,b,sp_img,pos[0],pos[1]);
	}

	std::vector<cv::Point2d> ppix;
	std::vector<cv::Point2d> mpix;
	
	cv::Mat iH = H.inv();

	for(int i = 0;i<img.rows;i++)
		for(int j = 0;j<img.cols;j++)
			ppix.push_back(cv::Point2d(j,i));

	mpix.resize(ppix.size());

	cv::perspectiveTransform(cv::Mat(ppix),cv::Mat(mpix),iH);

	for(size_t i = 0;i<mpix.size();i++)
	{
		int px = (int)(ppix[i].x);
		int py = (int)(ppix[i].y);
		
		double mx = mpix[i].x;
		double my = mpix[i].y;
		
		if(mx >= 0 && my >=0
			&& mx < img.cols 
			&& my < img.rows)
		{
			int r,g,b;
			LibIV::CppHelpers::Global::imgData(r,g,b,sp_img,(int)mx,(int)my);
			if(r != 0 || g != 0 || b != 0)
				LibIV::CppHelpers::Global::setImgData(r,g,b,img_show,px,py);
		}
	}

	/*// simple mapping
	std::vector<v2d> spc;
	NRDCShow::HomographyTransform(SP[sp_idx],H,spc);

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		v2d posc = spc[i];

		if(posc[0] >= 0 && posc[0] < img->width &&
			posc[1] >= 0 && posc[1] < img->height)
			cvSet2D(img_show,(int)(posc[1]),(int)(posc[0]),cvGet2D(img,pos[1],pos[0]));
	}*/
}
void NRDCShow::ShowSPc(IplImage * img, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H, const char * filename)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	NRDCShow::ShowSPc(img,img_show,SP,sp_idx,H);

	cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);
}

void NRDCShow::ShowSPc(cv::Mat & img, cv::Mat & img_show, const SuperPixels & SP, const std::vector<int> & sp_idx_vec, cv::Mat & H)
{
	img_show  = cv::Mat(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));
	for(size_t i = 0;i<sp_idx_vec.size();i++)
		NRDCShow::ShowSPc(img,img_show,SP,sp_idx_vec[i],H);	
}

void NRDCShow::ShowSPc(cv::Mat & img, cv::Mat & img_show, const SuperPixels & SP, const std::vector<int> & sp_idx_vec,  std::vector<cv::Mat> & H)
{
	img_show  = cv::Mat(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));
	for(size_t i = 0;i<sp_idx_vec.size();i++)
	{
		std::cout<<i<<std::endl;
		NRDCShow::ShowSPc(img,img_show,SP,sp_idx_vec[i],H[i]);
	}
}

void NRDCShow::ShowSPc(IplImage * img, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx, const char * filename)
{
	if(sp_idx.empty()) return;

	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	cv::Mat H;
	NRDCProcessing::ComputeHomographyForSP(nrdc,SP[sp_idx[0]],H);

	for(size_t i = 0;i<sp_idx.size();i++)
		NRDCShow::ShowSPc(img,img_show,SP,sp_idx[i],H);

	cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);
}

void NRDCShow::ShowSPc(IplImage * img, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, int sp_idx, const char * filename)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	for(size_t i = 0;i<SP[sp_idx].size();i++)
	{
		v2i pos = SP[sp_idx][i];
		v3d posc = nrdc.at(pos[1],pos[0]);

		if(posc[0] >= 0 && posc[0] < img->width &&
			posc[1] >= 0 && posc[1] < img->height)
			cvSet2D(img_show,(int)(posc[1]),(int)(posc[0]),cvGet2D(img,pos[1],pos[0]));
	}
	
	cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);
}

void NRDCShow::ShowSPc(IplImage * img, const std::vector<v2i> & sp, const std::vector<v2d> & spc, const char * filename)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];
		v2d posc = spc[i];
		

		if(posc[0] >= 0 && posc[0] < img_show->width &&
			posc[1] >= 0 && posc[1] < img_show->height)
			cvSet2D(img_show,(int)(posc[1]),(int)(posc[0]),cvGet2D(img,pos[1],pos[0]));
	}


	cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);
}



IplImage * NRDCShow::ShowSPc(IplImage * img, const std::vector<v2i> & sp, const std::vector<v2d> & spc, const char * filename, cv::Mat & H, int sp_idx)
{
	IplImage * img_show = cvCreateImage(cvSize(img->width,img->height),8,3);
	cvZero(img_show);

	for(size_t i = 0;i<sp.size();i++)
	{
		v2i pos = sp[i];
		v2d posc = spc[i];
		
		//if (posc[1] < 0 )
		//{
		//	double dd = 0;
		//	std::cout << H << std::endl;
		//	std::cout << sp_idx << std::endl;
		//}

		//if (posc[1] < -10000 )
		//{
		//	posc[1] = -posc[1];
		//	posc[0] = -posc[0];
		//}
		
		//posc[0] += img->width/2;
		//posc[1] += img->height/2;



		if(posc[0] >= 0 && posc[0] < img_show->width &&
			posc[1] >= 0 && posc[1] < img_show->height)
			cvSet2D(img_show,(int)(posc[1]),(int)(posc[0]),cvGet2D(img,pos[1],pos[0]));
	}

	return img_show;
	/*cvSaveImage(filename,img_show);
	cvReleaseImage(&img_show);*/
}