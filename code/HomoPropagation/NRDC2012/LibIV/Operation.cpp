//#include "stdafx.h"
#include "LibIVException.h"
#include "CppHelpers.h"
#include "Operation.h"

// -------------------------------------------------------
// Implementation of Operation3D

double LibIV::Operation::Operation3D::distance_three_face(VideoContent<CvScalar> * v1,int p1,int r1,int c1, 
														  VideoContent<CvScalar> * v2,int p2,int r2,int c2,int ns1,int ns2)
{
	double res = 0.0;
	int ii1,jj1,kk1;
	int ii2,jj2,kk2;
	CvScalar *** cs1,***cs2;
	double d;

	//static const double total = (ns1+ns2)*(ns1+ns2)*3 + (ns1+ns2)*3*10 + 50;

	cs1 = v1->pContent;
	cs2 = v2->pContent;

	for(int i = -ns1;i<=ns2;i++)
	{
		for(int j = -ns1;j<=ns2;j++)
		{
			if(i==0||j==0)
				continue;
			ii1 = p1;jj1 = r1 + i;kk1 = c1 + j;
			ii2 = p2;jj2 = r2 + i;kk2 = c2 + j;
			for(int k = 0;k<3;k++)
			{
				d = cs1[ii1][jj1][kk1].val[k] - cs2[ii2][jj2][kk2].val[k];
				res += d*d;
			}

			ii1 = p1 + i;jj1 = r1;kk1 = c1 + j;
			ii2 = p2 + i;jj2 = r2;kk2 = c2 + j;
			for(int k = 0;k<3;k++)
			{
				d = cs1[ii1][jj1][kk1].val[k] - cs2[ii2][jj2][kk2].val[k];
				res += d*d;
			}

			ii1 = p1 + i;jj1 = r1 + j;kk1 = c1;
			ii2 = p2 + i;jj2 = r2 + j;kk2 = c2;
			for(int k = 0;k<3;k++)
			{
				d = cs1[ii1][jj1][kk1].val[k] - cs2[ii2][jj2][kk2].val[k];
				res += d*d;
			}
		}				
	}

	//double res1 = res;
	for(int i = -ns1;i<=ns2;i++)
	{
		if(i==0)
			continue;
		for(int k = 0;k<3;k++)
		{
			d = cs1[p1+i][r1][c1].val[k] - cs2[p2+i][r2][c2].val[k];
			res += d*d;
			d = cs1[p1][r1+i][c1].val[k] - cs2[p2][r2+i][c2].val[k];
			res += d*d;
			d = cs1[p1][r1][c1+i].val[k] - cs2[p2][r2][c2+i].val[k];
			res += d*d;
		}
	}

	//double res2 = res;
	for(int k = 0;k<3;k++)
	{
		d = cs1[p1][r1][c1].val[k] - cs2[p2][r2][c2].val[k];
		res += d * d;
	}

	//double res3 = res;

	return sqrt(res);
}

// -------------------------------------------------------
// Implementation of Wrap3D
void LibIV::Operation::Wrap3D::wrap_trilinear(VideoContent<CvScalar> & vcdst, const VideoContent<CvScalar> & vcsrc)
{
	LIBIV_BEGIN

	int srcw,srcp,srch,dstw,dsth,dstp;
	double scale_w,scale_h,scale_p;
	int di,di1,di2,dj,dj1,dj2,dk1,dk2;

	srcw = vcsrc.getWidth();	srch = vcsrc.getHeight();
	srcp = vcsrc.getPage();		dstw = vcdst.getWidth();
	dsth = vcdst.getHeight();	dstp = vcdst.getPage();

	if(!srcw || !srch || !srcp || !dstw || !dsth || !dstp)
		LIBIV_EXCEPTION_WITH_ERROR(OperationException,"vcdst or vcsrc is NULL");

	scale_w = (double)dstw/srcw;
	scale_h = (double)dsth/srch;
	scale_p = (double)dstp/srcp;

	if(scale_w >= 1.0 && scale_h >= 1.0 && scale_p >= 1.0)
	{

		ForIndex(i,srcp)
		{
			// ------------------------------------------
			// Every line of the src video
			if(i==0) di = 0;
			else
				di = cvRound((i+1)*scale_p) - 1;
			ForIndex(j,srch)
			{
				if(j==0) dj = 0;
				else
					dj = cvRound((j+1)*scale_h) - 1;
				ForIndex(k,srcw-1)
				{
					// ----------------------------------
					// Get current dst's locations
					if(k==0) dk1=0;
					else
						dk1 = cvRound((k+1)*scale_w) - 1;
					dk2 = cvRound((k+2)*scale_w) - 1;
					//-----------------------------------
					// Assign these dst's pixels
					vcdst.pContent[di][dj][dk1] = vcsrc.pContent[i][j][k];
					vcdst.pContent[di][dj][dk2] = vcsrc.pContent[i][j][k+1];

					//-----------------------------------
					// Interpolation
					ForRange(r,dk1+1,dk2-1)
						ForIndex(v,3)
						vcdst.pContent[di][dj][r].val[v] = (vcdst.pContent[di][dj][dk1].val[v] * (dk2-r) + 
						vcdst.pContent[di][dj][dk2].val[v]*(r-dk1)) / (dk2-dk1);

				}
			}
			// -------------------------------------------
			// Every column of the dst video
			ForIndex(j,srch-1)
			{
				if(j==0) dj1 = 0;
				else
					dj1 = cvRound((j+1)*scale_h) - 1;
				dj2 = cvRound((j+2)*scale_h) - 1;
				// ----------------------------------------
				// Interpolation
				ForIndex(k,dstw)
				{
					ForRange(r,dj1+1,dj2-1)
						ForIndex(v,3)
						vcdst.pContent[di][r][k].val[v] = (vcdst.pContent[di][dj1][k].val[v] * (dj2-r) + 
						vcdst.pContent[di][dj2][k].val[v] * (r-dj1)) / (dj2-dj1);							
				}
			}
		}
		// ------------------------------------------------
		// The other pages
		ForIndex(i,srcp-1)
		{
			if(i==0) di1 = 0;
			else
				di1 = cvRound((i+1)*scale_p) - 1;
			di2 = cvRound((i+2)*scale_p) - 1;

			ForIndex(j,dsth)
				ForIndex(k,dstw)
				ForRange(r,di1+1,di2-1)
				ForIndex(v,3)
				vcdst.pContent[r][j][k].val[v] = (vcdst.pContent[di1][j][k].val[v] *(di2-r) + 
				vcdst.pContent[di2][j][k].val[v] * (r - di1))/ (di2-di1);
		}
	}
	else if(scale_w <= 1.0 && scale_h <= 1.0 && scale_p <= 1.0)
	{
		double scale_w_rev = (double)srcw/dstw;
		double scale_h_rev = (double)srch/dsth;
		double scale_p_rev = (double)srcp/dstp;
		ForIndex(i,dstp)
		{
			int si = cvRound((i+1)*scale_p_rev) - 1;
			ForIndex(j,dsth)
			{
				int sj = cvRound((j+1)*scale_h_rev)-1;
				ForIndex(k,dstw)
					vcdst.pContent[i][j][k] = vcsrc.pContent[si][sj][cvRound((k+1)*scale_w_rev)-1];
			}
		}
	}
	else
	{
		LIBIV_EXCEPTION_WITH_ERROR(OperationException,"The scaling is not symmetrical!");
	}
	LIBIV_END
}

void LibIV::Operation::Wrap3D::wrap_xy(VideoContent<CvScalar> & vcdst,const VideoContent<CvScalar> & vcsrc)
{
	if(!(vcsrc.getPage() == vcdst.getPage()))
		LIBIV_EXCEPTION_WITH_ERROR(OperationException,"The videos are different at pages' number.");
	
	int p, rd,cd,rs,cs;
	IplImage * ipls,*ipld;
	
	// Scale every page
	
	p = vcdst.getPage();
	rd = vcdst.getHeight();
	cd = vcdst.getWidth();
	rs = vcsrc.getHeight();
	cs = vcsrc.getWidth();

	ipls = cvCreateImage(cvSize(cs,rs),IPL_DEPTH_8U,3);
	ipld = cvCreateImage(cvSize(cd,rd),IPL_DEPTH_8U,3);

	ForIndex(i,p)
	{
		ForIndex(j,rd)
			ForIndex(k,cd)
				cvSet2D(ipld,j,k,vcdst.pContent[i][j][k]);
		ForIndex(j,rs)
			ForIndex(k,cs)
				cvSet2D(ipls,j,k,vcsrc.pContent[i][j][k]);
		
		cvResize(ipls,ipld,CV_INTER_AREA);
		
		ForIndex(j,rd)
			ForIndex(k,cd)
				vcdst.pContent[i][j][k] = cvGet2D(ipld,j,k);
	}
}