#ifndef NRDC_SHOW_H_
#define NRDC_SHOW_H_

#include "nrdc_common.h"

class NRDCShow
{
public:
	// show
	static void		ShowSP		(IplImage * img, const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx, const char * filename);
	static void		ShowSP		(IplImage * img, IplImage * img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, double alpha);
	static void		ShowSP		(IplImage * img, const std::vector<std::vector<v2i>> & SP, int sp_idx, const char * filename);
	static void		ShowSP		(IplImage * img, const std::vector<v2i> & sp, const char * filename);

	static void		ShowSPc		(IplImage * img, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx, const char * filename);
	static void		ShowSPc		(IplImage * img, IplImage * img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H); 
	static void		ShowSPc		(cv::Mat  & img, cv::Mat  & img_show, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H); 
	static void		ShowSPc		(cv::Mat  & img, cv::Mat  & img_show, const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx_vec, cv::Mat & H); 
	static void		ShowSPc		(cv::Mat  & img, cv::Mat  & img_show, const std::vector<std::vector<v2i>> & SP, const std::vector<int> & sp_idx_vec, std::vector<cv::Mat> & H);
	static void		ShowSPc		(IplImage * img, const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat & H, const char * filename);
	static void		ShowSPc		(IplImage * img, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, int sp_idx, const char * filename);
	static IplImage *     ShowSPc		(IplImage * img, const std::vector<v2i> & sp, const std::vector<v2d> & spc, const char * filename, cv::Mat & H, int sp_idx);
	static void     ShowSPc		(IplImage * img, const std::vector<v2i> & sp, const std::vector<v2d> & spc, const char * filename);
	static void		ShowSPc		(IplImage * img, IplImage * img_show, const std::vector<v2i> & sp, const std::vector<v2d> & spc);
};
#endif