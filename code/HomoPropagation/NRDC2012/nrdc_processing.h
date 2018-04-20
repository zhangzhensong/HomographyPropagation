#ifndef NRDC_PROCESSING_H_
#define NRDC_PROCESSING_H_

#include "nrdc_common.h"
#include "Warp\CommonStructures.h"
#include <string>

//#define ONLYRELIABLE 1
#define __FORWARD 1
//#define __BACKWARD 1


class NRDCProcessing
{
public:
	// tools
	static void		ConvertAscii2Binary				(const char * filename1, const char * filename2);
	static void 	BilinearInterpWeights			(double x, double y, double & w00, double & w10, double & w01, double & w11);
	static bool 	BilinearInterpImage				(double x, double y, double & r, double & g, double & b, IplImage * img);
	static bool 	BilinearInterpImage2			(double x, double y, double & r, double & g, double & b, const cv::Mat & img);
	static CvScalar ImageGet2D						(IplImage * img, double x, double y);
	static void     ImageGet2D						(int & r, int & g, int & b, const cv::Mat & img, double x, double y);
	static bool		ImageGet2D						(IplImage * img, double x, double y, CvScalar & cs);

	// prepare data
	static void		ReadNRDC						(NRDC & nrdc, const char * filename);
	static void		ReadSP							(const char * filename,std::vector<std::vector<v2i>> & SP);

    // save parameters
 

    static void     SaveIntermediateVariables(
        std::string fileName,
        std::vector<cv::Mat> &homoS,
        std::vector<int> &validHomoS);

    static void     LoadIntermediateVariables(
        std::string fileName,
        std::vector<cv::Mat> &homoS,
        std::vector<int> &validHomoS);
	
	// superpixel graph
	static SpGraph *	MakeSpGraph					(const std::vector<std::vector<v2i>> & SP,int width, int height);
	static void			DeleteSpGraph				(SpGraph ** graph);
	static void			ClearGraphTmp				(SpGraph * graph);
	static void			CollectN_Neighbors			(SpGraph * graph, int sp_idx, int N, std::vector<v2d> & neighbors);
	static void			ChiSquareDistOfEdge			(SpGraph * graph, const Histograms & hists, SpChiDiff & sp_chi_diff);
	static void			DijkstraShortestPath		(SpGraph * graph, const SpChiDiff & sp_chi_diff, int source_sp_idx, SpNode *& path_tree);
	static void			DijkstraShortestPath2		(SpGraph * graph, const SpChiDiff & sp_chi_diff, int source_sp_idx, SpNode *& path_tree);

	// superpixel coherence
	static double	ComputeCoherenceBetweenPixels	(v2i u, v2i v, const NRDC & nrdc);
	static double	SPCoherence						(const NRDC & nrdc, const std::vector<v2i> & sp);

	// find support region
	static void		ComputeHomographyForSP				(const NRDC & nrdc, const std::vector<v2i> & sp, cv::Mat & H);
	static void		ComputeHomographyForSP				(const NRDC & nrdc, const SuperPixels & SP, const std::vector<int> & seeds, cv::Mat & H);
	static void		DrawHomoWarp						(const NRDC & nrdc,const std::vector<v2i> & sp, cv::Mat & H, int spid, int width, int height);
	
	static void		HomographyTransform					(const std::vector<v2i> & sp, cv::Mat & H, std::vector<v2d> & spc);
	static double	ComputeDifferenceBetweenSPandSPC	(IplImage * img, IplImage * imgc, const std::vector<v2i> & sp, const std::vector<v2d> & spc);
	static void		FindSupportRegionGlobalTraversal	(IplImage * img_src, IplImage * img_dst, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, int sp_idx, std::vector<int> & sp_support, std::vector<double> & sp_support_diff);
	static void		FindSupportRegionWithinN_Neighbors	(IplImage * img_src, IplImage * img_dst, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, SpGraph * graph, int sp_idx, std::vector<v2d> & sp_support, cv::Mat & H);
	static void		FindSupportRegionWithinN_Neighbors  (IplImage * img_src, IplImage * img_dst, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, int sp_idx, std::vector<v2d> & sp_support, cv::Mat & H);
	static void		IsolateSupportRegion				(SpGraph * graph, std::vector<v2d> & sp_support); 

    // check homography
    static double   ComputeDisplacementOfSuperpixel (const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat propogatedH);
    static bool     CheckIfPropogatedHomographyReliable (const std::vector<std::vector<v2i>> & SP, int sp_idx, cv::Mat propogatedH, double thres);

	// histogram
	static void		ComputeHistogramForSPs			(const cv::Mat & img, const SuperPixels & SP, Histograms & hists);
	static void		ComputeHistogramForSpLab		(const cv::Mat & img_lab, const SuperPixels & SP, int sp_idx, Histogram & hist);
	static void		ComputeHistogramForSpRgb		(const cv::Mat & img, const SuperPixels & SP, int sp_idx, Histogram & r_hist, Histogram & g_hist, Histogram & b_hist);
	static double	ChiSquareDistBtwHist			(const Histogram & lhs_hist, const Histogram & rhs_hist);
	static double	ChiSquareDistBtwSp				(const Histograms & hists, int lhs_sp_idx, int rhs_sp_idx);

	// superpixel homography inference
	static void		DivideSpCategory							(SpGraph * graph, const SuperPixels & SP, const NRDC & nrdc, std::vector<int> & sp_cate_label);
	static void		ComputeNearestBoundarySpForMissingSp		(SpGraph * graph, const SpChiDiff & sp_chi_diff, const Histograms & hists, const std::vector<int> & sp_cate_label, const std::vector<int> & sp_mask, int missing_sp_idx, int & nearest_idx);
	static void		ComputeSeedsForHomographyInference			(SpGraph * graph, const SpChiDiff & sp_chi_diff, const std::vector<int> & sp_cate_label, const std::vector<int> & sp_mask, int nearest_boundary_idx, std::vector<int> & seeds);	
	static void		HistogramMatching							(const cv::Mat & img_src, const SuperPixels & SP, const std::vector<int> & seeds, int sp_idx, std::vector<v2i> & samples);

	// view interpolation
	static void		InitSeeds						(std::vector<int> & seeds, const std::vector<std::vector<v2i>> & SP, const NRDC & nrdc, int width, int height);
	static void		ViewInterpolation				(const std::vector<int> & seeds, IplImage * img_src, IplImage * img_dst, const NRDC & nrdc, const std::vector<std::vector<v2i>> & SP, SpGraph * graph);
	static void		ViewInterpolation				(std::string data_filepath, const cv::Mat & img_src, const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, int label_num);
	static void		ViewInterpolation				(std::string store_path, int label_num, const cv::Mat & img_forward, const cv::Mat & img_backward, const LibIV::Memory::Array::FastArray2D<v3d> & corre_forward, const LibIV::Memory::Array::FastArray2D<v3d> & corre_backward);
    static void		ViewInterpolation				(std::string store_path, int label_num, const cv::Mat & img_forward, const LibIV::Memory::Array::FastArray2D<v3d> & corre_forward, bool isForward);

	// forward warping and rendering
	static void		LocalWarping                    (const cv::Mat & img_src, const SuperPixels & SP, const NRDC & nrdc, SpGraph * graph, const Histograms & hists, const SpChiDiff & sp_chi_diff, const std::vector<int> & sp_mask, const std::vector<int> & sp_cate_label, cv::Mat & img_local, std::vector<Triangle> & all_triangles, std::vector<Vertex> & all_vertices, std::vector<Vertex> & all_deformed_vertices);
	static void		LocalWarping2                   (const cv::Mat & img_src, const SuperPixels & SP, const NRDC & nrdc, SpGraph * graph, const Histograms & hists, const SpChiDiff & sp_chi_diff, const std::vector<int> & sp_mask, const std::vector<int> & sp_cate_label, LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, int mask_id);
	static void		InverseMapping					(const cv::Mat & img_src, cv::Mat & img_show, cv::Mat & img_hole, const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre,const std::vector<v2i> & coord, const std::vector<v2d> & coordc);
	static void		InverseMapping					(const cv::Mat & img_src, cv::Mat & img_show, cv::Mat & img_hole, const LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre, int label_num);

	// bidirectional warping and rendering
	static void		ComputeSrcDstCompleteCorre		(std::string data_filepath, const std::vector<int> & segmentation_label, cv::Mat & img_src, LibIV::Memory::Array::FastArray2D<v3d> & src_dst_corre);
};

void findMaskedSuperpixels(const cv::Mat& unKnownMaskImg, const std::vector<std::vector<v2i>>& SP, std::vector<size_t>& maskedSuperpixels);

#endif