#ifndef _HELPERS_H
#define _HELPERS_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

struct PipelineOutput {
    cv_bridge::CvImagePtr image;
    double position;
    double radiusLeft;
    double radiusRight;
};

cv::Mat regionOfInterest(cv::Mat imageBinary, int mn, int mx);
cv::Mat scaleAbs(cv::Mat x, int m = 255);

#endif // _HELPERS_H
