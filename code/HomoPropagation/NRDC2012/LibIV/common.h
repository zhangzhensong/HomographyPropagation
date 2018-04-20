#pragma  once

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <assert.h>

#include <iostream>
#include <cmath>

using namespace std;

#define __DEBUG__

#ifdef __DEBUG__
#define _SHOW_(X) cout<<X<<endl;
#endif