#ifndef _HELPERS_H
#define _HELPERS_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

struct PipelineOutput {
    cv::Mat image;
    double position;
    double radiusLeft;
    double radiusRight;
};

struct CurvesResult {
    cv::Mat image;
    double leftRadius;
    double rightRadius;
    cv::Mat leftFitCurvePix;
    cv::Mat rightFitCurvePix;
    cv::Mat leftFitCurveF;
    cv::Mat rightFitCurveF;
    double vehiclePosition;
};

cv::Mat regionOfInterest(cv::Mat imageBinary, int mn, int mx);
cv::Mat scaleAbs(cv::Mat x, int m = 255);

#endif // _HELPERS_H
