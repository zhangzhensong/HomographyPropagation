#pragma once
#include "../nrdc_common.h"
#include <io.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>

class Preprocess
{
public:
    Preprocess(void);
    ~Preprocess(void);

    static void divideAccordingtoMask(
        const cv::Mat & mask,  
        std::vector<std::vector<v2i>> &SP, 
        std::vector<std::vector<v2i>> &newSP
                                      );
    static void divideAccordingtoMasks( 
        const std::vector<cv::Mat> &masks,
        std::vector<std::vector<v2i>> &SP, 
        std::vector<std::vector<v2i>> &newSP);

    static void mergeSuperpixelAccordingtoMasks(
        const std::vector<cv::Mat> &masks,
        std::vector<std::vector<v2i>> &SP
        );

    static void removeEmptySPs(std::vector<std::vector<v2i>> & SP);

    // Testing functions
    static void TestDivision(std::string filePath);
    static void TestPreprocess(
        std::string forward_dir_path,
        std::string backward_dir_path);
    static void TestMerging();

    static bool tellMeIfFileExist(const std::string filePath)
    {
        std::fstream _file;
        _file.open(filePath, std::ios::in);
        if (!_file)
        {
            _file.close();
            return false;
        }
        else
        {
            _file.close();
            return true;
        }
    }

    static bool isCvt(const std::string filePath)
    {
        //int handle;
        //handle = open(filePath.c_str(), 0x100);
        //long length = filelength(handle);
        //close(handle);
        //return length > 0;
        std::fstream infile(filePath, std::ios::binary);

        infile.seekg(0,std::ios::end);
        long long int ss = infile.tellg();
        return ss > 0;
    }
};

