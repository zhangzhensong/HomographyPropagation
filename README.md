# Homography Propagation and Optimization for Wide-baseline Street Image Interpolation

Demo code for the work below:
 - Nie, Y., Zhang, Z., Sun, H., Su, T., & Li, G. "[Homography Propagation and Optimization for Wide-baseline Street Image Interpolation](http://ieeexplore.ieee.org/document/7593383/)." IEEE Transactions on Visualization and Computer Graphics, 23(10), 2328-2341, 2017.

[Video Demo](https://www.youtube.com/watch?v=gZuqJWvDGOs&feature=youtu.be) on YouTube.

## External libraries and code used:
 - Boost 1.62: https://www.boost.org/
 - OpenCV 2.49: https://opencv.org/
 
## Run the code
1. The code is currently tested on VS 2012.
2. Configure the data path in the main.cpp file.
3. Define __FORWARD in nrdc_processing.h to run forward warping. The warping result will be saved in data/viewinterp/1.
4. Define __BACKWARD in nrdc_processing.h to run backward warping. The warping result will be saved in data/viewinterp/2. Note: DO not define __FORWARD and __BACKWARD at the same time.
5. Run the Matlab code in the folder multibandblending to render the final result, which will be save in data/viewterp/3.

## Common Questions
Q1: How to generate the labels.txt file in the forward/backward folder?

A1: This file provides an initial segmentation of the image. Any reasonable superpixel segmentation algorithm (For example, [SLIC](https://ivrl.epfl.ch/research/superpixels)) can be used. 

Q2: How to generate the matches.txt file in the forward/backward folder?

A2: This file can be generated from any ANNF algorithms, for example, [PatchMatch](http://gfx.cs.princeton.edu/pubs/Barnes_2010_TGP/index.php), [DAISY Filter Flow](https://sites.google.com/site/daisyfilterflowcvpr2014/) and [NRDC](http://www.cs.huji.ac.il/~yoavhacohen/nrdc/).

## Citation:

@article{nie2017homography,

  title={Homography propagation and optimization for wide-baseline street image interpolation},
  
  author={Nie, Yongwei and Zhang, Zhensong and Sun, Hanqiu and Su, Tan and Li, Guiqing},
  
  journal={IEEE transactions on visualization and computer graphics},
  
  volume={23},
  
  number={10},
  
  pages={2328--2341},
  
  year={2017},
  
  publisher={IEEE}
}
 
Code by Zhensong Zhang and Yongwei Nie. For research purpose ONLY. 