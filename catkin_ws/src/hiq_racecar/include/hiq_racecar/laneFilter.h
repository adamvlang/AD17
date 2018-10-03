#ifndef _LANE_FILTER_H
#define _LANE_FILTER_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudaarithm.hpp"

class LaneFilter {
public:
    LaneFilter(int saturationThreshold,
               int lightThreshold,
               int lightThresholdArg,
               double gradientThresholdLow,
               double gradientThresholdHigh,
               int magnitudeThreshold,
               int xThreshold);
    cv::Mat apply(cv::Mat rgbImage);

private:
    int saturationThreshold;
    int lightThreshold;
    int lightThresholdArg;
    double gradientThresholdLow;
    double gradientThresholdHigh;
    int magnitudeThreshold;
    int xThreshold;
    cv::Mat hls;
    cv::Mat l;
    cv::Mat s;
    cv::Mat z;
    cv::Mat colorCond1;
    cv::Mat colorCond2;
    cv::Mat sobelCond1;
    cv::Mat sobelCond2;
    cv::Mat sobelCond3;

    cv::Mat sobelBreakdown(cv::Mat image);
    cv::Mat colorBreakdown(cv::Mat image);
    cv::Mat applyColorMask();
    cv::Mat applySobelMask();
};

#endif _LANE_FILTER_H
