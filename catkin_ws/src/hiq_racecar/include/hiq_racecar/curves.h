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
    void storeDetails(const cv::Mat binary);
    void start(const cv::Mat binary, int *currentLeftX, int *currentRightX);
    void nextY(const int w, int *lowY, int *highY);
    void nextX(const int current, int *leftX, int rightX);
    void nextMidX(const int pixelIndices[], int *current);
    void drawBoundaries(const cv::Point2f p1, const cv::Point2f p2, const Scalar& color, int thickness = 5);
    void indicesWithinBoundary(const int lowY, const int highY, const int leftX, const int rightX, cv::Mat returnMat);
    void pixelLocations(const int indices[], int allPixelsX[], int allPixelsY[]);
    void plot(int t = 4);
    void getRealCurvature(const int xs[], const int ys[], double coefficients[]);
    void radiusOfCurvature(const double y, const double coefficients[], double *radius);
    void updateVehiclePosition(double *vehiclePosition);
    void covertPointToArray(vector<Point>, int[], int[]);

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
