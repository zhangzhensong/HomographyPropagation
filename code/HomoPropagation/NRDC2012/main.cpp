#include "nrdc_test.h"
#include "nrdc_processing.h"
#include "Preprocess/Preprocess.h"
#include "Pipeline.h"

#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <Eigen\Dense>

using namespace std;

int main()
{
	std::string work_dir_path =  "E:/democase/data/";
	std::string forward_dir_path = work_dir_path + "forward/";
	std::string backward_dir_path = work_dir_path + "backward/";
	
	Preprocess::TestPreprocess(forward_dir_path, backward_dir_path);

	Pipeline::TestBidirectional(work_dir_path, forward_dir_path, backward_dir_path);
	Pipeline::TestReadCorre(work_dir_path, forward_dir_path, backward_dir_path);

	system("pause");
	return 0;
}