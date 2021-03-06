#ifndef _CURVES_H
#define _CURVES_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudaarithm.hpp"

#include <math.h>

#include "helpers.h"

using namespace std;

class Curves {
public:
    Curves();
    Curves(int numberOfWindows,
           int margin,
           int minPix,
           double ymPerPixel,
           double xmPerPixel);
    CurvesResult fit(cv::Mat binary);

private:
    void storeDetails(const cv::Mat binary);
    void start(const cv::Mat binary, int *currentLeftX, int *currentRightX);
    void nextY(const int w, int *lowY, int *highY);
    void nextX(const int current, int *leftX, int *rightX);
    void nextMidX(const int pixelIndices[], int *current);
    void drawBoundaries(const cv::Point2f p1, const cv::Point2f p2, const cv::Scalar& color, int thickness = 5);
    void indicesWithinBoundary(const int lowY, const int highY, const int leftX, const int rightX, vector<int> returnVector);
    void pixelLocations(const vector<int> indices, cv::Mat pixelsX, cv::Mat pixelsY, int length);
    void plot(int t = 4);
    void getRealCurvature(cv::Mat xs, cv::Mat ys, cv::Mat coefficients);
    void radiusOfCurvature(const double y, const cv::Mat coefficients, double *radius);
    void updateVehiclePosition();
    void convertPointToArray(vector<cv::Point> locations);
    void polyfit(const cv::Mat &srcX, const cv::Mat &srcY, cv::Mat &dst, int order);

    int minPix;
    int margin;
    int numberOfWindows;
    double xmPerPixel;
    double ymPerPixel;

    cv::Mat binary;
    cv::Mat outImg;

    int h;
    int w;
    int windowHeight;
    int mid;

    vector<int> allPixelsX;
    vector<int> allPixelsY;

    cv::Mat leftPixelsX;
    cv::Mat rightPixelsX;
    cv::Mat leftPixelsY;
    cv::Mat rightPixelsY;

    cv::Mat leftFitCurvePix;
    cv::Mat rightFitCurvePix;
    cv::Mat leftFitCurveF;
    cv::Mat rightFitCurveF;

    double leftRadius;
    double rightRadius;
    double vehiclePosition;
};

#endif // _CURVES_H
