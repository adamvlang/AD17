#ifndef _CURVES_H
#define _CURVES_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudaarithm.hpp"

#include "helpers.h"

class Curves {
public:
    Curves();
    CurvesResult fit(cv::Mat binary);

private:
    void storeDetails(cv::Mat binary);


    int minPix;
    double margin;
    int numberOfWindows;
    double xmPerPixel;
    double ymPerPixel;

    cv::Mat binary;
    cv::Mat OutImg;

    int h;
    int w;
    int windowHeight;
    int mid;

    int allPixelsX[];
    int allPixelsY[];

    bool leftPixelsX[];
    bool rightPixelsX[];
    bool leftPixelsY[];
    bool rightPixelsY[];
    bool leftPixelIndices[];
    bool rightPixelIndices[];

    double leftFitCurvePix[];
    double rightFitCurvePix[];
    double leftFitCurveF[];
    double rightFitCurveF[];

    double leftRadius;
    double rightRadius;
    double vehiclePosition;
};

#endif // _CURVES_H
