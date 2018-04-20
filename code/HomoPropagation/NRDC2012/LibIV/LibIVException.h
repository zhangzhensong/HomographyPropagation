/*
	Well, everything must start at somewhere!
	
	LibIV : Library for Image and Video processing.
	This is just a simple and private library for the easy of 
	image & video programming.
	Many codes of this library come from  Sylvain Lefebvre ' LibSL.
	If you are interested in his works,please refer to 
	http://www-sop.inria.fr/members/Sylvain.Lefebvre/_wiki_/pmwiki.php?n=Main.TSynEx

	================================================
	LibIVException        2009/10/31
	================================================
	
	A base class for exception processing
*/

#pragma once

#include <iostream>
#include <cstdio>
#include <cstdarg>

namespace LibIV
{
	namespace Exception
	{
		class Exception
		{
		public:
			enum { e_MessageBufferSize = 4096 };

		protected:

			// -----------------------------------------
			// The message!
			char m_Message[e_MessageBufferSize];

		public:
			// -----------------------------------------
			// Constructor
			Exception()													{}
			Exception(const char * msg,...)
			{
				va_list args;
				va_start(args,msg);
				vsprintf_s(m_Message,e_MessageBufferSize,msg,args);
				va_end(args);
			}
			
			// -----------------------------------------
			// Derived class should override this function
			virtual void whatMsg()	const								{ std::cerr<<"<<<Base exception>>>\n"<<m_Message<<std::endl; }

			// -----------------------------------------
			//A constant function
			const char * message() const                                { return m_Message; }

		};//End of Exception

		class OperationException : public Exception
		{
		public:
			OperationException(const char * msg,...) 
			{
				va_list args;
				va_start(args,msg);
				vsprintf_s(m_Message,e_MessageBufferSize,msg,args);
				va_end(args);
			}
			
			virtual void whatMsg()  const								{ std::cerr<<"<<<OperationException>>>\n"<<m_Message<<std::endl; }

		};//End of 3DWrapException

	}//End of Exception
	
	
}//End of LibIV

// -----------------------------------------------------

#define LIBIV_EXCEPTION(M)												throw LibIV::Exception::M("file: %s\nline: %d\n",__FILE__,__LINE__)
#define LIBIV_EXCEPTION_WITH_ERROR(M,E)									throw LibIV::Exception::M(E"\nfile: %s\nline: %d\n",__FILE__,__LINE__)
#define LIBIV_BOUNCE_EXCEPTION(E)										throw LibIV::Exception::Exception("%s\n[caller] file: %s\nline: %d\n",E.message(),__FILE__,__LINE__)
