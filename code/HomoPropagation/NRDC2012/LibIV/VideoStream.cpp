
#include <LibIV/VideoStream.h>
#include <LibIV/CppHelpers.h>

using namespace LibIV::Video;

VideoStream::VideoStream() : m_nWidth(0),m_nHeight(0),m_nPages(0),m_nChannels(0),m_dFps(0.0),m_Img(NULL)
{
}

VideoStream::~VideoStream() 
{
	if(!m_Img)
		cvReleaseImage(&m_Img);
}

IplImage * const VideoStream::getImage(int frame)
{
	return NULL;
}

IplImage * const VideoStream::getImage()
{
	return NULL;
}

IplImage * const VideoStream::getFront()
{
	return NULL;
}

IplImage * const VideoStream::getNext()
{
	return NULL;
}

void VideoStream::seek(VideoStream::POSF pos)
{
	if(pos == BEGIN)
		getImage(shift.Be());
	else if(pos == END)
		getImage(shift.Ed());
}

IVideoStream_VdoCap::IVideoStream_VdoCap(const char *filename)
{
	m_pCapture = cvCreateFileCapture(filename);
	_check_(m_pCapture);

	m_nWidth = (int)cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_WIDTH);
	m_nHeight = (int)cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_HEIGHT);
	m_nPages = (int)cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_COUNT);
	m_dFps = cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FPS);
	m_nChannels = 3;

	shift.Init(0,m_nPages-1);
	
	m_Img = cvCreateImage(cvSize(m_nWidth,m_nHeight),IPL_DEPTH_8U,m_nChannels);
	_check_(m_Img);
}

IVideoStream_VdoCap::~IVideoStream_VdoCap()
{
	if(m_pCapture)
		cvReleaseCapture(&m_pCapture);
}

IplImage * const IVideoStream_VdoCap::getImage(int frame)
{
	_check_r_(frame>=0 && frame<m_nPages && m_pCapture && m_Img);

	cvSetCaptureProperty(m_pCapture,CV_CAP_PROP_POS_FRAMES,frame);
	IplImage * ipl = cvQueryFrame(m_pCapture);
	cvCopy(ipl,m_Img);
	shift.Scp(frame);

	return m_Img;
}

IplImage * const IVideoStream_VdoCap::getImage()
{
	getImage(shift.Cp());
	return m_Img;
}
IplImage * const IVideoStream_VdoCap::getFront()
{
	_check_r_(shift.Back() && m_pCapture && m_Img);

	cvSetCaptureProperty(m_pCapture,CV_CAP_PROP_POS_FRAMES,shift.Cp());
	IplImage * ipl = cvQueryFrame(m_pCapture);
	cvCopy(ipl,m_Img);

	return m_Img;
}

IplImage * const IVideoStream_VdoCap::getNext()
{
	_check_r_(shift.Next() && m_pCapture && m_Img);

	cvSetCaptureProperty(m_pCapture,CV_CAP_PROP_POS_FRAMES,shift.Cp());
	IplImage * ipl = cvQueryFrame(m_pCapture);
	cvCopy(ipl,m_Img);

	return m_Img;
}

IVideoStream_ImgSeq::IVideoStream_ImgSeq(const char * filepath, const char * filename, const char * extname, int fb,int fe)
{
	char name[1024];
	sprintf_s(name,1024,"%s/%s%04d.%s",filepath,filename,fb,extname);

	IplImage * ipl = cvLoadImage(name);
	_check_(ipl);

	strcpy_s(m_filepath,1024,filepath);
	strcpy_s(m_filename,1024,filename);
	strcpy_s(m_appendix,128,extname);

	shift.Init(fb,fe);

	m_nWidth = ipl->width;
	m_nHeight = ipl->height;
	m_nPages = fe-fb+1;
	m_nChannels = ipl->nChannels;
	m_dFps = 29.3;
	
	m_Img = cvCreateImage(cvSize(m_nWidth,m_nHeight),IPL_DEPTH_8U,m_nChannels);
	_check_(m_Img);

	cvReleaseImage(&ipl);
}

IVideoStream_ImgSeq::~IVideoStream_ImgSeq()
{

}

IplImage * const IVideoStream_ImgSeq::getImage(int frame)
{
	_check_r_(frame >= shift.Be() && frame <= shift.Ed() && m_Img);

	char name[1024];
	sprintf_s(name,1024,"%s/%s%04d.%s",m_filepath,m_filename,frame,m_appendix);

	IplImage * ipl = cvLoadImage(name);
	cvCopy(ipl,m_Img);

	shift.Scp(frame);

	cvReleaseImage(&ipl);

	return m_Img;
}

IplImage * const IVideoStream_ImgSeq::getImage()
{
	getImage(shift.Cp());
	return m_Img;
}

IplImage * const IVideoStream_ImgSeq::getNext()
{
	_check_r_(shift.Next() && m_Img);

	return getImage(shift.Cp());
}

IplImage * const IVideoStream_ImgSeq::getFront()
{
	_check_r_(shift.Back() && m_Img);

	return getImage(shift.Cp());
}