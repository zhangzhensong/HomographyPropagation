#include "Preprocess.h"
#include "../nrdc_processing.h"

#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <map>

Preprocess::Preprocess(void)
{
}


Preprocess::~Preprocess(void)
{
}


void Preprocess::divideAccordingtoMask( const cv::Mat & mask,  
                                        std::vector<std::vector<v2i>> &SP, 
                                        std::vector<std::vector<v2i>> &newSP)
{
    int numOfSuperPixels = SP.size();
    std::vector<bool> bCoherence(numOfSuperPixels, false);

    for (int i = 0; i < SP.size(); ++i)
    {
        int nCountPositive = 0;
        for (int j = 0; j < SP[i].size(); ++j)
        {
            if (mask.at<uchar>(SP[i][j][1], SP[i][j][0]) > 128)
                ++nCountPositive;
        }

        int minNum = 1;

        if (nCountPositive < minNum || nCountPositive > SP[i].size() - minNum)
        {
            bCoherence[i] = true;
        }
    }

    int numOfIncoherenceSuperPixels = 0;
    for (int i = 0; i < bCoherence.size(); ++i)
    {
        if (!bCoherence[i])
        {
            ++numOfIncoherenceSuperPixels;
        }
    }

    newSP.clear();
    newSP.resize(SP.size() + numOfIncoherenceSuperPixels);

    for (int i = 0; i < newSP.size(); i++)
    {
        newSP[i].clear();
    }

    int newIndex = SP.size();
    for (int i = 0; i < SP.size(); ++i)
    {
        for (int j = 0; j < SP[i].size(); ++j)
        {

            if (!bCoherence[i])
            {
                if (mask.at<uchar>(SP[i][j][1], SP[i][j][0]) > 128)
                {
                    newSP[newIndex].push_back(SP[i][j]);
                }else
                {
                    newSP[i].push_back(SP[i][j]);
                }
            }else
            {
                newSP[i].push_back(SP[i][j]);
            }            
        }

        if (!bCoherence[i])
        {
            ++newIndex;
        }
    }
}

void Preprocess::divideAccordingtoMasks(
    const std::vector<cv::Mat> &masks,
    std::vector<std::vector<v2i>> &SP, 
    std::vector<std::vector<v2i>> &newSP)
{
    for (int i = 0; i < masks.size(); ++i)
    {
        divideAccordingtoMask(masks[i], SP, newSP);

        SP.clear();
        SP.resize(newSP.size());
        for (int j = 0; j < newSP.size(); ++j)
        {
            SP[j].clear();
            for (int k = 0; k < newSP[j].size(); ++k)
            {
                SP[j].push_back(newSP[j][k]);
            }
        }
    }
}

void convertSPto2DArray(const std::vector<std::vector<v2i>> &SP, 
    const int nImgWidth, 
    const int nImgHeight, 
    LibIV::Memory::Array::FastArray2D<int>& sp_label_array)
{
    sp_label_array.set(nImgWidth, nImgHeight);
    sp_label_array.fill(0);

    for(size_t i = 0;i<SP.size();i++)
    {
        for(size_t j = 0;j<SP[i].size();j++)
        {
            v2i pos = SP[i][j];
            sp_label_array.at(pos[1],pos[0]) = (int)i;
        }
    }
}

void Preprocess::TestDivision(std::string filePath)
{
    //std::string filePath = "D:/test20150716/";
    std::string imgPath = filePath + "a.bmp";

    cv::Mat image_src = cv::imread(imgPath);
    int imgWidth = image_src.cols;
    int imgHeight = image_src.rows;    
    
    std::vector<std::vector<v2i>> SP, newSP;
    NRDCProcessing::ReadSP((filePath + "SP.txt").c_str(), SP);

    cv::Mat image_mask_tree = cv::imread(filePath + "mask_car.bmp", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat image_mask_background = cv::imread(filePath + "mask_background.bmp", CV_LOAD_IMAGE_GRAYSCALE);

    std::cout << image_mask_tree.cols << std::endl;
    std::cout << image_mask_tree.rows << std::endl;

    std::vector<cv::Mat> masks;
    masks.clear();

    masks.push_back(image_mask_tree);
    masks.push_back(image_mask_background);

    int originSPNum = SP.size();

    divideAccordingtoMask(image_mask_tree, SP, newSP);
    //divideAccordingtoMasks(masks, SP, newSP);
    std::cout << " 2 Final size is " << newSP.size() << std::endl;

    mergeSuperpixelAccordingtoMasks(masks, newSP);
    std::cout << "Final size (after merge) size is " << newSP.size() << std::endl;

    // 2d array of sp labels
    LibIV::Memory::Array::FastArray2D<int> sp_label_array;
    convertSPto2DArray(newSP, imgWidth, imgHeight, sp_label_array);

    std::ofstream out((filePath + "newSP.txt").c_str());
    out << imgWidth << " " << imgHeight << " " << newSP.size() << " " << originSPNum << std::endl;
    for (int i = 0; i < imgHeight; ++i)
    {
        for (int j = 0; j < imgWidth; ++j)
        {
            out << sp_label_array.at(i, j) << std::endl;
        }
    }
    out.close();
}

void subPreprocess(std::string folderPath)
{
    assert(Preprocess::tellMeIfFileExist(folderPath + "labels.txt"));
    assert(Preprocess::tellMeIfFileExist(folderPath + "matches.txt"));

    if (!Preprocess::isCvt(folderPath + "NRDC.txt"))
    {
        NRDCProcessing::ConvertAscii2Binary((folderPath + "matches.txt").c_str(),
            (folderPath + "NRDC.txt").c_str());
    }
    
    NRDCProcessing::ConvertAscii2Binary((folderPath + "labels.txt").c_str(),
        (folderPath + "SP.txt").c_str());

    if (Preprocess::tellMeIfFileExist(folderPath + "mask_car.bmp") && Preprocess::tellMeIfFileExist(folderPath + "mask_background.bmp"))
    {
        Preprocess::TestDivision(folderPath);
        NRDCProcessing::ConvertAscii2Binary((folderPath + "newSP.txt").c_str(),
            (folderPath + "SP.txt").c_str());
    }
}

void Preprocess::TestPreprocess(std::string forward_dir_path,
                                std::string backward_dir_path)
{
    subPreprocess(forward_dir_path + "/");
    subPreprocess(backward_dir_path + "/");
}

int findSPMaskID( const int querySPID, 
    const std::vector<std::vector<v2i>> &SP, 
    const std::vector<cv::Mat> &masks)
{
    std::vector<int> maskCount(masks.size(), 0);
    for (int i = 0; i < SP[querySPID].size(); ++i)
    {
        v2i pos = SP[querySPID][i];

        for (int j = 0; j < masks.size(); ++j)
        {
            if (masks[j].at<uchar>(pos[1], pos[0]) > 128)
            {
                ++maskCount[j];
            }
        }
    }

    int maxID = -1;
    for (int i = 0; i < maskCount.size(); ++i)
    {
        if (maskCount[i] > maxID)
        {
            maxID = i;
        }
    }

    return maxID;
}


// need improve
int findNeighborSameMaskSP(const int querySPID, 
                           const std::vector<std::vector<v2i>> &SP, 
                           const LibIV::Memory::Array::FastArray2D<int> &sp_label_array,
                           const std::vector<cv::Mat> &masks)
{
    for (int i = 0; i < masks.size(); ++i)
    {
        assert(masks[i].channels() == 1);
    }

    // step 1, check belongs to which mask
    int maskID = findSPMaskID(querySPID, SP, masks);

    // step 2, find neighboring superpixel that belongs to the same mask
    const int width = sp_label_array.cols();
    const int height = sp_label_array.rows();

    std::map<int, int> mapii;
    mapii.clear();


    for (int i = 0; i < SP[querySPID].size(); ++i)
    {
        v2i pos = SP[querySPID][i];
        int x = pos[0], y = pos[1];

        if (x > 0)
        {
            int b = sp_label_array.at(y, x - 1);
            if (b != querySPID && masks[maskID].at<uchar>(y, x - 1) > 128)
            {
                std::map<int, int>::iterator iter = mapii.find(b);
                if (iter != mapii.end())
                {
                    mapii[b]++;
                }
                else
                {
                    mapii[b] = 1;
                }
            }
        }

        if (x < width - 1)
        {
            int b = sp_label_array.at(y, x + 1);
            if (b != querySPID && masks[maskID].at<uchar>(y, x + 1) > 128)
            {
                std::map<int, int>::iterator iter = mapii.find(b);
                if (iter != mapii.end())
                {
                    mapii[b]++;
                }
                else
                {
                    mapii[b] = 1;
                }
            }
        }

        if (y > 0)
        {
            int b = sp_label_array.at(y - 1, x);
            if (b != querySPID && masks[maskID].at<uchar>(y - 1, x) > 128)
            {
                std::map<int, int>::iterator iter = mapii.find(b);
                if (iter != mapii.end())
                {
                    mapii[b]++;
                }
                else
                {
                    mapii[b] = 1;
                }
            }
        }

        if (y < height - 1)
        {
            int b = sp_label_array.at(y + 1, x);
            if (b != querySPID && masks[maskID].at<uchar>(y + 1, x) > 128)
            {
                std::map<int, int>::iterator iter = mapii.find(b);
                if (iter != mapii.end())
                {
                    mapii[b]++;
                }
                else
                {
                    mapii[b] = 1;
                }
            }
        }
    }

    int candidateSPID = -1;
    int maxnum = -1;
    for (std::map<int, int>::iterator it = mapii.begin(); it != mapii.end(); it++)
    {
        if (it->second > maxnum)
        {
            maxnum = it->second;
            candidateSPID = it->first;
        }
    }

    return candidateSPID;

    //for (int i = 0; i < SP[querySPID].size(); ++i)
    //{
    //    v2i pos = SP[querySPID][i];
    //    int x = pos[0], y = pos[1];

    //    if (x > 0)
    //    {
    //        int b = sp_label_array.at(y, x - 1);
    //        if (b != querySPID && masks[maskID].at<uchar>(y, x - 1) > 128)
    //        {
    //            return b;
    //        }
    //    }

    //    if (x < width - 1)
    //    {
    //        int b = sp_label_array.at(y, x + 1);
    //        if (b != querySPID && masks[maskID].at<uchar>(y, x + 1) > 128)
    //        {
    //            return b;
    //        }
    //    }

    //    if (y > 0)
    //    {
    //        int b = sp_label_array.at(y - 1, x);
    //        if (b != querySPID && masks[maskID].at<uchar>(y - 1, x) > 128)
    //        {
    //            return b;
    //        }
    //    }

    //    if (y < height - 1)
    //    {
    //        int b = sp_label_array.at(y + 1, x);
    //        if (b != querySPID && masks[maskID].at<uchar>(y + 1, x) > 128)
    //        {
    //            return b;
    //        }
    //    }
    //}

    return -1;
}

void Preprocess::TestMerging()
{
    std::string filePath = "D:/test20150716/";
    std::vector<std::vector<v2i>> SP, newSP;
    NRDCProcessing::ReadSP((filePath + "SP.txt").c_str(), SP);
}

void Preprocess::removeEmptySPs(std::vector<std::vector<v2i>> & SP)
{
    //std::vector<std::vector<v2i>> newSP;
    //newSP.clear();
    //for (int i = 0; i < SP.size(); ++i)
    //{
    //    if (SP[i].size() == 0)
    //        continue;

    //    std::vector<v2i> tmpSP;
    //    for (int j = 0; j < SP[i].size(); ++j)
    //    {
    //        tmpSP.push_back(SP[i][j]);
    //    }

    //    newSP.push_back(tmpSP);
    //}

    //SP.clear();
    //SP.resize(newSP.size());
    //for (int i = 0; i < newSP.size(); ++i)
    //{
    //    for (int j = 0; j < newSP[i].size(); ++j)
    //    {
    //        SP[i].push_back(newSP[i][j]);
    //    }
    //}

    std::vector<std::vector<v2i>>::iterator it = SP.begin();

    for (it = SP.begin(); it != SP.end();)
    {
        if (it->size() == 0)
        {
            it = SP.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void Preprocess::mergeSuperpixelAccordingtoMasks(
    const std::vector<cv::Mat> &masks,
    std::vector<std::vector<v2i>> &SP
    )
{
    const int nImgWidth = masks[0].cols;
    const int nImgHeight = masks[0].rows;
    
    LibIV::Memory::Array::FastArray2D<int> sp_label_array;
    convertSPto2DArray(SP, nImgWidth, nImgHeight, sp_label_array);
    
    
    // test neibor sp
    //int neiborID = findNeighborSameMaskSP(31, SP, sp_label_array, masks);
    //assert(neiborID != -1);
    //std::cout << "NeighborID = " << neiborID << std::endl;


    const int minPixelNum = 200;
    
    int iterNum = 0;

    while (true)
    {
        std::cout << "Merge iteration is " << iterNum++ << std::endl;
        for (int i = 0; i < SP.size(); ++i)
        {
            if (SP[i].size() != 0 && SP[i].size() < minPixelNum)
            {
                int neighborID = findNeighborSameMaskSP(i, SP, sp_label_array, masks);
                assert(neighborID != -1);
                for (int j = 0; j < SP[i].size(); ++j)
                {
                    SP[neighborID].push_back(SP[i][j]);
                }
                SP[i].clear();
                convertSPto2DArray(SP, nImgWidth, nImgHeight, sp_label_array);
            }
        }

        Preprocess::removeEmptySPs(SP);

        bool bAllBigEnough = true;
        for (int i = 0; i < SP.size(); ++i)
        {
            if (SP[i].size() < minPixelNum)
            {
                bAllBigEnough = false;
                break;
            }
        }

        if (bAllBigEnough)
        {
            break;
        }
    }
}