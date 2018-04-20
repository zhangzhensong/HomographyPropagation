/********************************************************************
	============================================
	Created:	2010/12/27
	FileName: 	VideoStream.h
	Author:		Nie Yongwei
	Purpose:	Read and write the video as stream
	============================================
	
	Modified on 2011/2/22
	IVideoStream_ImgSeq and IVideoStream_VdoCap
	============================================
*********************************************************************/

#pragma  once
#include <opencv2/highgui/highgui.hpp>

//#include <cv.h>
//#include <cxcore.h>

namespace LibIV
{
	namespace Video
	{
		class Shift
		{
		public:
			Shift() { be = ed = 0; ToBe(); gap = 1;}
			
			void Init(int _be, int _ed) 
			{ 
				be = _be; 
				ed = _ed; 
				gap = 1;
				ToBe(); 
			}

			void Destroy()
			{
				be = ed = 0;
				gap = 0;
				ToBe();
			}
			
			~Shift() {}

			Shift& operator=(const Shift& s)
			{
				be = s.be;
				ed = s.ed;
				cp = s.cp;
				gap = s.gap;
				return * this;
			}

			bool Next() { if(cp + gap >= be && cp + gap <= ed) { cp += gap; return true; } else return false;}
			bool Back() { if(cp - gap >= be && cp - gap <= ed) { cp -= gap; return true; } else return false;}
			void ToBe() { cp = be; }
			void ToEd() { cp = ed; }
			
			
			int Cp(void) const { return cp; }
			int Be(void) const { return be; }
			int Ed(void) const { return ed; }
			int Gap(void) const { return gap; }

			void Scp(int n) { cp = n; }
			void Sbe(int n) { be = n; }
			void Sed(int n) { ed = n; }
			void SGap(int n) { gap = n; }

			int Intervel(void) const { return (ed-be+1); }

		private:
			int cp;
			int be;
			int ed;
			int gap;
		};

		class VideoStream
		{
		public:
			typedef enum { BEGIN, END } POSF;
			VideoStream();
			virtual ~VideoStream();
			
			// ------------------------------------------------------------------------------ //
			//get information
			int getPage(void)   const { return m_nPages;    }
			int getWidth(void)  const { return m_nWidth;    }
			int getHeight(void) const { return m_nHeight;   }
			double getFps(void) const { return m_dFps;      }
			int getChannels(void)   const { return m_nChannels; }
			
			void setPage(int n)       { m_nPages = n; }
			void setWidth(int n)      { m_nWidth = n; }
			void setHeight(int n)     { m_nHeight = n;}
			void setChannels(int n)   { m_nChannels = n; }
			void setFps(double n)     { m_dFps = n; }

			virtual IplImage * const getImage(int frame);
			virtual IplImage * const getImage();
			virtual IplImage * const getFront();
			virtual IplImage * const getNext();
			virtual void             writeFrame(IplImage * img) {}

			void seek(VideoStream::POSF pos);
			
		public:
			Shift shift;
		protected:
			int m_nWidth,m_nHeight,m_nPages, m_nChannels;
			double m_dFps;
			IplImage * m_Img;

		
		};

		// The stream from a video capture
		class IVideoStream_VdoCap : public VideoStream
		{
		public:
			IVideoStream_VdoCap(const char * filename);
			~IVideoStream_VdoCap();

			virtual IplImage * const getImage(int frame);
			virtual IplImage * const getImage();
			virtual IplImage * const getFront();
			virtual IplImage * const getNext ();

		private:
			CvCapture * m_pCapture;
		};

		// The stream from image sequences
		class IVideoStream_ImgSeq : public VideoStream
		{
		public:
			IVideoStream_ImgSeq(const char * filepath, const char * filename, const char * extname, int fb,int fe);
			~IVideoStream_ImgSeq();

			virtual IplImage * const getImage(int frame);
			virtual IplImage * const getImage();
			virtual IplImage * const getFront();
			virtual IplImage * const getNext();

		private:
			char m_filepath[1024];
			char m_filename[1024];
			char m_appendix[128];
		};

		class OVideoStream : public VideoStream
		{
		public:
			OVideoStream(const char * filepath, int width, int height, double fps);
			~OVideoStream();
			
			virtual void writeFrame(IplImage * img);
		private:
			CvVideoWriter * pWriter;
		};
	}
}