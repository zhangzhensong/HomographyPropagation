/*
	===========================
	VideoContent	09/10/30
	===========================
	A new version of VideoContent.	

	Modified on 2010/11/26
	The base element can be int or float or CvScalar and any 
	other types. 
	The template is used and combined with strategy pattern.

	Modified on 2010/12/22
	Load video from frames.

	Modified on 2010/12/27
	Add to namespace LibIV
*/

#pragma once
//#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <LibIV/videostream.h>


namespace LibIV
{
	namespace Video
	{
		template<typename T_Type>
		class VideoContent;

		inline void IntToRGB(int pixel, int & r, int & g, int & b)
		{
			r = (pixel&255);
			g = (pixel>>8)&255;
			b = pixel>>16;
		}

		inline int RGBToInt(int r,int g,int b)
		{
			return (int)((b<<16)|(g<<8)|r);
		}


		// ------------------------------------------------------------------------------ //
		// the strategy pattern
		class VideoContent_Strategy
		{
		protected:
			VideoContent_Strategy();
		public:
			virtual ~VideoContent_Strategy();

			virtual bool read(const char * filename, const void * pvc) = 0;
			virtual bool read(const char * filename, int bp, int ep, const void * pvc) = 0;
			virtual bool save(const char * filename, const void * pvc) = 0;
			virtual IplImage* frame(const void * pvc, int n) = 0;
			virtual void fill(const void * pvc, void * val) = 0;
		};

		// ------------------------------------------------------------------------------ //
		// int
		class VideoContent_Strategy_int : public VideoContent_Strategy
		{
		public:
			VideoContent_Strategy_int();
			~VideoContent_Strategy_int();

			virtual bool read(const char * filename, const void * pvc);
			virtual bool read(const char * filename, int bp, int ep, const void * pvc);
			virtual bool save(const char * filename, const void * pvc);
			virtual IplImage* frame(const void * pvc, int n);
			virtual void fill(const void * pvc, void * val);
		};

		// ------------------------------------------------------------------------------ //
		// CvScalar
		class VideoContent_Strategy_CvScalar : public VideoContent_Strategy
		{
		public:
			VideoContent_Strategy_CvScalar();
			~VideoContent_Strategy_CvScalar();

			virtual bool read(const char * filename, const void * pvc);
			virtual bool read(const char * filename, int bp, int ep, const void * pvc);
			virtual bool save(const char * filename, const void * pvc);
			virtual IplImage* frame(const void * pvc, int n);
			virtual void fill(const void * pvc, void * val);
		};


		template<typename T_Type>
		class VideoContent
		{
		public:
			// ------------------------------------------------------------------------------ //
			//Initialize a VideoContent. No content.
			VideoContent();												
			VideoContent(int width,int height,int page,int channels,double fps);		

			// ------------------------------------------------------------------------------ //
			//Copy constructor.
			VideoContent(const VideoContent & vc);
			VideoContent & operator=(const VideoContent & vc);

			// ------------------------------------------------------------------------------ //
			~VideoContent(void);
		public:
			// ------------------------------------------------------------------------------ //
			// create the video
			void create(int width,int height,int page,int channels,double fps);

			// ------------------------------------------------------------------------------ //
			// read and save the video
			bool readVideo(const char * fileName);
			bool saveVideo(const char * fileName);

			// read from frames
			bool readVideo(const char * fileName, int bp, int ep);

			// add n at the end of filename
			bool saveVideo(const char * fileName,int n);
			// save the frames that from b to e
			bool saveVideo(const char * fileName,int b,int e);

			// ------------------------------------------------------------------------------ //
			//get information
			int getPage(void)   const { return m_nPages;    }
			int getWidth(void)  const { return m_nWidth;    }
			int getHeight(void) const { return m_nHeight;   }
			double getFps(void) const { return m_dFps;      }
			int getChannels()   const { return m_nChannels; }

			void setPage(int n)       { m_nPages = n; }
			void setWidth(int n)      { m_nWidth = n; }
			void setHeight(int n)     { m_nHeight = n;}
			void setChannels(int n)   { m_nChannels = n; }
			void setFps(double n)     { m_dFps = n; }

			void getParameters(int& width, int &height,int &page,int& channel ,double& fps) const
			{ width = m_nWidth;height=m_nHeight;page = m_nPages;channel=m_nChannels; fps = m_dFps; }

			// ------------------------------------------------------------------------------ //
			// get the p-th frame
			IplImage* getFrame(int p);

			// ------------------------------------------------------------------------------ //
			// clear 
			void cleanUp(void);
			void fill(T_Type val);

			// ------------------------------------------------------------------------------ //
			//The video, a 3-dimension array.
			T_Type *** pContent;

		public:

			// ------------------------------------------------------------------------------ //
			//Memory operations
			T_Type *** mem_alloc(int p,int h,int w);
			void mem_free(T_Type **** p);
			void mem_copy(T_Type *** dst, T_Type *** src, unsigned int max_count);

		private:
			int m_nWidth, m_nHeight, m_nPages, m_nChannels;
			double m_dFps;

			// ------------------------------------------------------------------------------ //
			// different T_Type should be processed differently
			VideoContent_Strategy * pVCS;

		public:
			Shift shift;
		};

		// ------------------------------------------------------------------------------ //
		// IMPLEMENTION : VideoContent

		template<typename T_Type>
		VideoContent<T_Type>::VideoContent():pContent(NULL),m_nWidth(0),m_nHeight(0),m_nPages(0),m_nChannels(0),m_dFps(0.0)
		{
			if(sizeof(T_Type) == sizeof(int))
			{
				pVCS = new VideoContent_Strategy_int();
			}
			else
			{
				pVCS = new VideoContent_Strategy_CvScalar();
			}
		}

		template<typename T_Type>
		VideoContent<T_Type>::VideoContent(int width,int height,int page,int channels,double fps)
		{
			pContent = NULL;

			if(sizeof(T_Type) == sizeof(int))
			{
				pVCS = new VideoContent_Strategy_int();
			}
			else
			{
				pVCS = new VideoContent_Strategy_CvScalar();
			}

			create(width,height,page,channels,fps);
			shift.Init(0,page-1);
		}

		template<typename T_Type>
		VideoContent<T_Type>::VideoContent(const VideoContent & vc)
		{
			assert(vc.pContent);

			// clear, pVCS not clear
			cleanUp();

			m_nWidth = vc.m_nWidth;m_nHeight = vc.m_nHeight;
			m_nPages = vc.m_nPages;m_nChannels = vc.m_nChannels;
			m_dFps = vc.m_dFps;

			shift = vc.shift;
 
			pContent = mem_alloc(m_nPages,m_nHeight,m_nWidth);

			mem_copy(pContent,vc.pContent,sizeof(T_Type)*m_nPages*m_nHeight*m_nWidth);
		}

		template<typename T_Type>
		void VideoContent<T_Type>::create(int width,int height,int page,int channels,double fps)
		{
			// clear
			cleanUp();

			m_nWidth = width;
			m_nHeight = height;
			m_nPages = page;
			m_nChannels = channels;
			m_dFps = fps;

			pContent = mem_alloc(m_nPages,m_nHeight,m_nWidth);
			memset(pContent[0][0],0,sizeof(T_Type) * m_nPages * m_nHeight * m_nWidth);
		}

		template<typename T_Type>
		VideoContent<T_Type> & VideoContent<T_Type>::operator=(const VideoContent & vc)
		{
			assert(vc.pContent);
			cleanUp();

			m_nWidth = vc.m_nWidth;m_nHeight = vc.m_nHeight;
			m_nPages = vc.m_nPages;m_nChannels = vc.m_nChannels;
			m_dFps = vc.m_dFps;

			shift = vc.shift;

			pContent = mem_alloc(m_nPages,m_nHeight,m_nWidth);

			mem_copy(pContent,vc.pContent,sizeof(T_Type)*m_nPages*m_nHeight*m_nWidth);

			return (*this);
		}

		template<typename T_Type>
		T_Type *** VideoContent<T_Type>::mem_alloc(int p,int h,int w)
		{
			T_Type *** a = new T_Type**[p];
			a[0] = new T_Type*[p*h];
			a[0][0] = new T_Type[p*h*w];

			for(int i = 1;i<p;i++)
			{
				a[i] = a[i-1] + h;
				a[i][0] = a[i-1][0] + w * h;
			}

			for(int i = 0;i<p;i++)
			{
				for(int j = 1;j<h;j++)
				{
					a[i][j] = a[i][j-1] + w;
				}
			}
			return a;
		}

		template<typename T_Type>
		void VideoContent<T_Type>::mem_free(T_Type **** p)
		{
			if(!p || !*p)
				return;
			delete [] (*p)[0][0];
			delete [] (*p)[0];
			delete [] (*p);
			(*p) = NULL;
		}

		template<typename T_Type>
		void VideoContent<T_Type>::mem_copy(T_Type *** dst, T_Type *** src, unsigned int max_count)
		{
			assert(dst&&src);
			memcpy(dst[0][0],src[0][0],max_count);
		}

		template<typename T_Type>
		VideoContent<T_Type>::~VideoContent(void)
		{
			cleanUp();
			if(pVCS)
				delete pVCS;
		}

		template<typename T_Type>
		void VideoContent<T_Type>::cleanUp()
		{
			if(pContent)
				mem_free(&pContent);
			m_nWidth = 0;
			m_nHeight = 0;
			m_nChannels = 0;
			m_nPages = 0;
			m_dFps = 0.0;
			shift.Destroy();
		}

		template<typename T_Type>
		void VideoContent<T_Type>::fill(T_Type val)
		{
			if(pVCS)
				pVCS->fill(this,&val);
		}

		template<typename T_Type>
		bool VideoContent<T_Type>::readVideo(const char * fileName)
		{
			if(pVCS)
				return pVCS->read(fileName,this);
			return false;
		}

		template<typename T_Type>
		bool VideoContent<T_Type>::readVideo(const char * fileName, int bp, int ep)
		{
			if(pVCS)
				return pVCS->read(fileName,bp,ep,this);
			return false;
		}

		template<typename T_Type>
		bool VideoContent<T_Type>::saveVideo(const char * fileName)
		{
			if(pVCS)
				return pVCS->save(fileName,this);
			return false;
		}

		template<typename T_Type>
		bool VideoContent<T_Type>::saveVideo(const char * fileName,int n)
		{
			char _fileName[1024];
			char _cNum[256];
			sprintf_s(_cNum,1024,"%d",n);

			int len = 0;
			int len2;
			while(fileName[len] != '\0')
			{
				len++;
			}
			if(len <= 4)
				return false;
			len2 = len;
			len--;
			while(fileName[len] != '.')
			{
				len--;
			}
			int lenn = 0;
			while(_cNum[lenn] != '\0')
			{
				lenn++;
			}

			for(int i = 0;i<len;i++)
				_fileName[i] = fileName[i];
			for(int i = 0;i<lenn;i++)
				_fileName[i+len] = _cNum[i];
			for(int i = len;i<len2;i++)
				_fileName[lenn + i] = fileName[i];

			_fileName[len2 + lenn] = '\0';

			return saveVideo(_fileName);
		}

		template<typename T_Type>
		bool VideoContent<T_Type>::saveVideo(const char * fileName,int b,int e)
		{
			return false;
		}

		template<typename T_Type>
		IplImage * VideoContent<T_Type>::getFrame(int p)
		{
			if(pVCS)
				return pVCS->frame(this,p);
			return NULL;
		}
	}
}
