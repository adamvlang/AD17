#include "curves.h"

using namespace std;

Curves::Curves(int numberOfWindows, double margin, int minPix, double ymPerPixel, double xmPerPixel) {
    this->minPix = minPix;
    this->margin = margin;
    this->numberOfWindows = numberOfWindows;
    this->xmPerPixel = xmPerPixel;
    this->ymPerPixel = ymPerPixel;
}

Curves::Curves() {


}

void Curves::storeDetails(const cv::Mat binary){
    std::vector<cv::Mat> channels;
    channels.push_back(binary*255);
    channels.push_back(binary*255);
    channels.push_back(binary*255);
    merge(channels, this->outImg);
    this->binary = binary;
    this->h = binary.size().height;
    this->w = binary.size().width;
    this->mid = static_cast<int>(this->h / 2);
    this->windowHeight = static_cast<int>(this->h / this->numberOfWindows);
    vector<cv::Point> locations;
    cv::findNonZero(binary, locations);

    covertPointToArray(locations, this->allPixelsX, this->allPixelsY);
}

void Curves::covertPointToArray(vector<cv::Point> locations, int allPixelsX[], int allPixelsY[]){
    int count = locations.size();
    int pixelsX[count] = {0};
    int pixelsY[count] = {0};
    for(int i = 0; i < count; i++) {
        cv::Point point = locations[i];
        pixelsX[i] = point.x;
        pixelsY[i] = point.y;
    }

    allPixelsX = pixelsX;
    allPixelsY = pixelsY;
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
