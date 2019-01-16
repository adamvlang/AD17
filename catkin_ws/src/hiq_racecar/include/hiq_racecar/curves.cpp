#include "curves.h"

Curves::Curves(int numberOfWindows, double margin, int minPix, double ymPerPixel, double xmPerPixel) {
    this->minPix = minPix;
    this->margin = margin;
    this->numberOfWindows = numberOfWindows;
    this->xmPerPixel = xmPerPixel;
    this->ymPerPixel = ymPerPixel;
}

Curves::Curves() {

}

void Curves::start(const cv::Mat binary, int *currentLeftX, int *currentRightX) {

}

void Curves::nextY(const int w, int *lowY, int *highY) {

}

void Curves::nextX(const int current, int *leftX, int rightX) {

}

void Curves::nextMidX(const int pixelIndices[], int *current) {

}

void Curves::drawBoundaries(const cv::Point2f p1, const cv::Point2f p2, const cv::Scalar& color, int thickness) {

}

void Curves::indicesWithinBoundary(const int lowY, const int highY, const int leftX, const int rightX, cv::Mat returnMat) {

}

void Curves::pixelLocations(const int indices[], int allPixelsX[], int allPixelsY[]) {

}

void Curves::plot(int t) {

}

void Curves::getRealCurvature(const int xs[], const int ys[], double coefficients[]) {

}

void Curves::radiusOfCurvature(const double y, const double coefficients[], double *radius) {

}

void Curves::updateVehiclePosition(double *vehiclePosition) {

}
