/*
	============================
	Operation     2009/10/31
	============================

	The operations on images and videos
*/

#pragma once
#include "VideoContent.h"

using namespace LibIV::Video;

namespace LibIV
{
	namespace Operation
	{
		/* ---------------------------------------------------------
			class Operation3D:
			The common 3D operations
			---------------------------------------------------------*/
		class Operation3D
		{
		public:
			// ---------------------------------------------------
			// The three faces are point(p1,r1,c1)'s neighborhood just like what it is be 
			// in Johannes Kopf' Solid Texture Synthesis.
			static double distance_three_face(VideoContent<CvScalar> * v1,int p1,int r1,int c1, 
								VideoContent<CvScalar> * v2,int p2,int r2,int c2,int ns1,int ns2);
		
		}; // End of Operation3D

		/* ---------------------------------------------------------
			class Wrap3D:
			Simple methods for 3-dimension data wrapping.

			Case 1:
			If the dst's size is bigger than src's , I use a trilinear-
			interpolation method.

			Case 2:
			Otherwise , I just pick-up some important pixels.	
		   --------------------------------------------------------- */
		class Wrap3D
		{
		public:
			// ---------------------------------------------------
			// Wrap(Resizing)             !!!VideoContent should be changed to a interior data structure
			static void wrap_trilinear(VideoContent<CvScalar> & vcdst,
				const VideoContent<CvScalar> & vcsrc);

			// ---------------------------------------------------
			// Just wrap width and height. The time axis retains.
			static void wrap_xy(VideoContent<CvScalar> & vcdst,const VideoContent<CvScalar>& vcsrc);

		};//End of Wrap3D

		
	}//End of Operation

}//End of LibIV




