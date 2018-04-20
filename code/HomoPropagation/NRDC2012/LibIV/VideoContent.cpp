//#include "stdafx.h"
#include <LibIV/VideoContent.h>

using namespace LibIV::Video;

VideoContent_Strategy::VideoContent_Strategy() {}

VideoContent_Strategy::~VideoContent_Strategy() {}

VideoContent_Strategy_int::VideoContent_Strategy_int() {}

VideoContent_Strategy_int::~VideoContent_Strategy_int() {}

bool VideoContent_Strategy_int::read(const char * filename, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<int> * p = (VideoContent<int> *)pvc;

	CvCapture * pCapture = cvCreateFileCapture(filename);
	if(!pCapture)
		return false;

	p->cleanUp();

	int width,height,page;
	double fps;

	width = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_WIDTH);
	height = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_HEIGHT);
	page = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_COUNT);
	fps = cvGetCaptureProperty(pCapture,CV_CAP_PROP_FPS);

	p->setWidth(width);
	p->setHeight(height);
	p->setPage(page);
	p->setFps(fps);

	p->shift.Init(0,page-1);

	p->pContent = p->mem_alloc(page,height,width);

	IplImage * ipl_tmp = NULL;
	CvScalar cs;
	int tmp;

	for(int i = 0;i<page;i++)
	{
		cvSetCaptureProperty(pCapture,CV_CAP_PROP_POS_FRAMES,i);
		ipl_tmp = cvQueryFrame(pCapture);
		
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
			{
				//cs = cvGet2D(ipl_tmp,height-j-1,k);
				cs = cvGet2D(ipl_tmp,j,k);

				tmp = (int)cs.val[0];
				tmp = tmp << 8;
				tmp = tmp | (int)cs.val[1];
				tmp = tmp << 8;
				tmp = tmp | (int)cs.val[2];
					
				p->pContent[i][j][k] = tmp;
			}
	}

	p->setChannels(ipl_tmp->nChannels);

	cvReleaseCapture(&pCapture);
	return true;
}

bool VideoContent_Strategy_int::read(const char * filename, int bp, int ep, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<int> * p = (VideoContent<int> *)pvc;

	p->cleanUp();
	
	int width,height,page;
	const double fps = 23;
	char buf[1024];
	IplImage * img;

	sprintf_s(buf,1024,"%s%03d.png",filename,bp);
	img = cvLoadImage(buf);
	width = img->width;
	height = img->height;
	page = ep-bp+1;

	p->shift.Init(bp,ep);
	
	p->setWidth(width);
	p->setHeight(height);
	p->setPage(page);
	p->setFps(fps);
	p->setChannels(img->nChannels);
	cvReleaseImage(&img);

	p->pContent = p->mem_alloc(page,height,width);

	CvScalar cs;
	int tmp;

	for(int i = 0;i<page;i++)
	{
		sprintf_s(buf,1024,"%s%03d.png",filename,i+bp);
		img = cvLoadImage(buf);
	
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
			{
				cs = cvGet2D(img,j,k);

				tmp = (int)cs.val[0];
				tmp = tmp << 8;
				tmp = tmp | (int)cs.val[1];
				tmp = tmp << 8;
				tmp = tmp | (int)cs.val[2];

				p->pContent[i][j][k] = tmp;
			}

		cvReleaseImage(&img);
	}

	return true;
}

bool VideoContent_Strategy_int::save(const char * filename, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<int> * p = (VideoContent<int> *)pvc;

	if(!p->pContent)
		return false;

	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	CvVideoWriter * pWriter = cvCreateVideoWriter(filename,/*CV_FOURCC('X','V','I','D')*/0, fps,cvSize(width,height),1);

	if(!pWriter)
		return false;

	IplImage * ipl_tmp = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,channel);
	//ipl_tmp->origin = 1;

	CvScalar cs;
	int tmp;

	for(int i = 0;i<page;i++)
	{
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
			{
				tmp = p->pContent[i][j][k];
				cs.val[2] = tmp&255;
				cs.val[1] = (tmp>>8)&255;
				cs.val[0] = tmp>>16;
				cvSet2D(ipl_tmp,j,k,cs);
			}
		cvWriteFrame(pWriter,ipl_tmp);
	}
	cvReleaseImage(&ipl_tmp);
	cvReleaseVideoWriter(&pWriter);
	return true;
}

IplImage* VideoContent_Strategy_int::frame(const void * pvc, int n)
{
	if(!pvc)
		return false;
	VideoContent<int> * p = (VideoContent<int> *)pvc;

	if(!p->pContent)
		return NULL;

	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	if(n<0 || n >= page)
		return NULL;

	IplImage * presult = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,channel);
	CvScalar cs;
	int tmp;

	for(int i = 0;i<height;i++)
		for(int j = 0;j<width;j++)
		{
			tmp = p->pContent[n][i][j];
			cs.val[2] = tmp&255;
			cs.val[1] = (tmp>>8)&255;
			cs.val[0] = tmp>>16;
			
			cvSet2D(presult,i,j,cs);
		}
	return presult;
}

void VideoContent_Strategy_int::fill(const void * pvc,void * val)
{
	if(!pvc)
		return;
	VideoContent<int> * p = (VideoContent<int> *)pvc;

	if(!p->pContent)
		return;

	int * pval = (int*)(val);

	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	for(int i = 0;i<page;i++)
	{
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
				p->pContent[i][j][k] = (*pval);
	}
}

VideoContent_Strategy_CvScalar::VideoContent_Strategy_CvScalar() {}

VideoContent_Strategy_CvScalar::~VideoContent_Strategy_CvScalar() {}

bool VideoContent_Strategy_CvScalar::read(const char * filename, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<CvScalar> * p = (VideoContent<CvScalar> *)pvc;
	
	CvCapture * pCapture = cvCreateFileCapture(filename);
	if(!pCapture)
		return false;

	p->cleanUp();

	int width,height,page;
	double fps;

	width = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_WIDTH);
	height = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_HEIGHT);
	page = (int)cvGetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_COUNT);
	fps = cvGetCaptureProperty(pCapture,CV_CAP_PROP_FPS);

	p->setWidth(width);
	p->setHeight(height);
	p->setPage(page);
	p->setFps(fps);

	p->shift.Init(0,page-1);
	
	p->pContent = p->mem_alloc(page,height,width);

	IplImage * ipl_tmp = NULL;

	for(int i = 0;i<page;i++)
	{
		cvSetCaptureProperty(pCapture,CV_CAP_PROP_POS_FRAMES,i);
		ipl_tmp = cvQueryFrame(pCapture);

		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
				p->pContent[i][j][k] = cvGet2D(ipl_tmp,height-j-1,k);
	}

	p->setChannels(ipl_tmp->nChannels);

	cvReleaseCapture(&pCapture);
	return true;
}

bool VideoContent_Strategy_CvScalar::read(const char * filename, int bp, int ep, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<CvScalar> * p = (VideoContent<CvScalar> *)pvc;

	p->cleanUp();

	int width,height,page;
	const double fps = 23;
	char buf[1024];
	IplImage * img;

	sprintf_s(buf,1024,"%s%03d.png",filename,bp);
	img = cvLoadImage(buf);
	width = img->width;
	height = img->height;
	page = ep-bp+1;

	p->shift.Init(bp,ep);

	p->setWidth(width);
	p->setHeight(height);
	p->setPage(page);
	p->setFps(fps);
	p->setChannels(img->nChannels);
	cvReleaseImage(&img);

	p->pContent = p->mem_alloc(page,height,width);

	
	for(int i = 0;i<page;i++)
	{
		sprintf_s(buf,1024,"%s%03d.png",filename,i+bp);
		img = cvLoadImage(buf);

		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
				p->pContent[i][j][k] = cvGet2D(img,j,k);;
			
			cvReleaseImage(&img);
	}

	return true;
}

bool VideoContent_Strategy_CvScalar::save(const char * filename, const void * pvc)
{
	if(!pvc)
		return false;
	VideoContent<CvScalar> * p = (VideoContent<CvScalar> *)pvc;

	if(!p->pContent)
		return false;

	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	CvVideoWriter * pWriter = cvCreateVideoWriter(filename,/*CV_FOURCC('X','V','I','D')*/0, fps,cvSize(width,height),1);
	
	if(!pWriter)
		return false;
	
	IplImage * ipl_tmp = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,channel);
	//ipl_tmp->origin = 1;

	for(int i = 0;i<page;i++)
	{
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
				cvSet2D(ipl_tmp,j,k,p->pContent[i][j][k]);

		cvWriteFrame(pWriter,ipl_tmp);
	}
	cvReleaseImage(&ipl_tmp);
	cvReleaseVideoWriter(&pWriter);
	return true;
}

IplImage* VideoContent_Strategy_CvScalar::frame(const void * pvc, int n)
{
	if(!pvc)
		return false;
	VideoContent<CvScalar> * p = (VideoContent<CvScalar> *)pvc;

	if(!p->pContent)
		return NULL;
		
	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	if(n<0 || n >= page)
		return NULL;

	IplImage * presult = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,channel);

	for(int i = 0;i<height;i++)
		for(int j = 0;j<width;j++)
			cvSet2D(presult,i,j,p->pContent[n][i][j]);
	
	return presult;
}

void VideoContent_Strategy_CvScalar::fill(const void * pvc, void * val)
{
	if(!pvc)
		return;
	VideoContent<CvScalar> * p = (VideoContent<CvScalar> *)pvc;

	if(!p->pContent)
		return;

	CvScalar * pcs = (CvScalar*)val;

	double fps;
	int width,height,page,channel;

	p->getParameters(width,height,page,channel,fps);

	for(int i = 0;i<page;i++)
	{
		for(int j = 0;j<height;j++)
			for(int k = 0;k<width;k++)
				p->pContent[i][j][k] = (*pcs);
	}
}