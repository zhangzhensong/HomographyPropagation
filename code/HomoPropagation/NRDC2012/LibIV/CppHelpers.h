/*
	=============================
	CppHelpers     2009/10/31
	=============================

	C++ language syntax helpers
*/

#pragma once

#include "LibIVException.h"
#include <string>
#include <sstream>
//#include <cv.h>

#define LIBIV_DEBUG


// -------------------------------------------------------------------------------------------------------------------------------
#define ForIndex(I,N)							for(int I = 0;      I< int(N); I++)
#define ForRange(I,A,B)							for(int I = (int)A; I<=int(B); I++)
#define ForRangeReverse(I,A,B)                  for(int I = (int)A; I>=int(B); I--)


// -------------------------------------------------------------------------------------------------------------------------------
#define LIBIV_BEGIN								{try{
#define LIBIV_END							    }catch(const LibIV::Exception::Exception& e){ LIBIV_BOUNCE_EXCEPTION(e);}}

#define LIBIV_APPLICATION_BEGIN                 {try{
#define LIBIV_APPLICATION_END  	                }catch(const LibIV::Exception::Exception& e){std::cout<<"---\n   "<<e.message()<<"\n      Caller: "<<__FILE__<<"\n         Line: "<<__LINE__<<std::endl;}}

// -------------------------------------------------------------------------------------------------------------------------------
#ifdef LIBIV_RELEASE
#	define iv_dbg_assert(B)
#else
#	define iv_dbg_assert(B)						if(!(B)) LIBIV_EXCEPTION_WITH_ERROR(Exception,"**ASSERT FAILED** " #B)
#endif


#ifdef LIBIV_DEBUG 
#define _error_  \
	std::cerr<<"\n---\n   check error at: "<<'\n' \
	<<"      file : "<<__FILE__<<'\n' \
	<<"      line : "<<__LINE__<<"\n      PLS CHECK IT!"<<std::endl
#else
#define _error_
#endif

#define _assert_(b) { if(!(b)) { _error_; exit(-1);}} 
#define _check_(b)  { if(!(b)) { _error_; return;}}
#define _check_r_(b){ if(!(b)) { _error_; return 0;}}
#define _check_b_(b){ if(!(b)) { _error_; return false;}}
#define _check_pt_(b){ if(!(b)) { _error_; return cvPoint(0,0);}}


namespace LibIV
{
	namespace CppHelpers
	{
		namespace Console
		{
#ifdef WIN32
			void pushCursor();
			void popCursor(bool clearLines = false);
#endif
			//! clear screen
			void clearScreen();
			//! print colored text
			std::ostream& red	(std::ostream& s);
			std::ostream& green (std::ostream& s);
			std::ostream& blue  (std::ostream& s);
			std::ostream& yellow(std::ostream& s);
			std::ostream& white (std::ostream& s);
			std::ostream& gray  (std::ostream& s);
		} // end of namespace Console
		
		namespace Global
		{
			std::string intToStr(int n);

			/* 
				Usage: Read optical flow computed by algorithm "Anisotropic Huber-L1 Optical Flow" from file.
				Params: 
					filename: the file name
					u: x direction
					v: y direction
					width: the width of image
					height: the height of image
					store_type: 0 - column by column, 1 - line by line 
			*/
			bool readFlow(const char * filename, float *& u, float *& v, int & width, int & height, int store_type = 0);

			/* 
				Usage: Get and set values for image.
			*/
			inline void imgData(int & p, IplImage * img, int x, int y)
			{
				p = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels];
			}
			
			inline void imgData(int & p, cv::Mat & img, int x, int y)
			{
				p = (int)((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels()];
			}

            inline void imgData(int & p, const cv::Mat & img, int x, int y)
            {
                p = (int)((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels()];
            }

			inline void imgData(double & p, IplImage * img, int x, int y)
			{
				p = ((double*)(img->imageData + img->widthStep * y))[x * img->nChannels];
			}

			inline void imgData(double & r, double & g, double & b, IplImage * img, int x, int y)
			{
				b = ((double*)(img->imageData + img->widthStep * y))[x * img->nChannels];
				g = ((double*)(img->imageData + img->widthStep * y))[x * img->nChannels + 1];
				r = ((double*)(img->imageData + img->widthStep * y))[x * img->nChannels + 2];
			}

			inline void imgData(int & r, int & g, int & b, IplImage * img, int x, int y)
			{
				b = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels];
				g = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 1];
				r = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 2];
			}

			inline void imgData(int & r, int & g, int & b,int & a, IplImage * img, int x, int y)
			{
				b = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels];
				g = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 1];
				r = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 2];
				a = (int)((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 3];
			}

			inline void imgData(int & r, int & g, int & b, const cv::Mat & img, int x, int y)
			{
				b = (int)((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels()];
				g = (int)((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels() + 1];
				r = (int)((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels() + 2];
			}

			inline void setImgData(int r, int g, int b, IplImage * img, int x, int y)
			{		
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels] = static_cast<uchar>(b);
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 1] = static_cast<uchar>(g);
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 2] = static_cast<uchar>(r);
			}

			inline void setImgData(int r, int g, int b, cv::Mat & img, int x, int y)
			{		
				((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels()] = static_cast<uchar>(b);
				((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels() + 1] = static_cast<uchar>(g);
				((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels() + 2] = static_cast<uchar>(r);
			}
			
			inline void setImgData(int g, cv::Mat & img, int x, int y)
			{		
				((uchar*)(img.data + img.channels() * img.cols * y))[x * img.channels()] = static_cast<uchar>(g);
			}
		
			inline void setImgData(int r, int g, int b, int a, IplImage * img, int x, int y)
			{
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels] = static_cast<uchar>(b);
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 1] = static_cast<uchar>(g);
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 2] = static_cast<uchar>(r);
				((uchar*)(img->imageData + img->widthStep * y))[x * img->nChannels + 3] = static_cast<uchar>(a);
			}

			template<class T> inline std::string num2str(T num)
			{
				std::ostringstream oss;
				oss<<num;
				return oss.str();
			}
			
		} // end of namespace Global	
	}
}