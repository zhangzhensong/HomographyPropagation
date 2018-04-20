#ifndef NRDC_TEST_H_
#define NRDC_TEST_H_

#include <string>
#include <vector>

class NRDCTest
{
public:
	// test
	static void		TestSingleSPProperty			();
	static void		TestComputeSupportRegion		();
	static void		TestViewInterpolation			();
	static void		TestHistogram					();
	static void		TestWarping						();
	static void		TestNearRegionHomoAccuracy		();
	static void		TestDijkstra					();
	static void		TestDivideSpCategory			();
	static void		TestNearestBoundarySp			();
	static void		TestAllHomograpies				();
	static void		TestOurWarping					();
	static void		TestCarWarping					();
	static void		TestLocalWarping				();
	static void		TestAllWarping					();
	static void		TestBidirectional				();
	static void		TestReadCorre					();
	static void		TestAllGround					();
	static void		TestHomography                  ();
	static void		TestImgAlignment				();

	static std::string folerpath;
    static bool isRunningBackward;
    static std::vector<int> missingRegion;

};

#endif