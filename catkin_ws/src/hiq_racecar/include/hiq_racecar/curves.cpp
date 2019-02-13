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

    convertPointToArray(locations, this->allPixelsX, this->allPixelsY);
}

void Curves::start(const cv::Mat binary, int *currentLeftX, int *currentRightX){
    cv::Mat bottom_half = binary(cv::Range(0, this->w), cv::Range((int)this->h/2, this->h));
    vector<int> hist;
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM); // TODO: Use CalcHist?

    int mid = (int)(hist.size() / 2);
    int length = hist.size();
    int indexRight = 0;
    int indexLeft = 0;
    int maxValue = 0;

    for(size_t i = 0; i < mid; i++)
    {
        int value = hist[i];
        if (value > maxValue) {
            indexLeft = i;
            maxValue = value;
        }
    }

    maxValue = 0;

    for(size_t i = mid + 1 ; i < length; i++)
    {
        int value = hist[i];
        if(value > maxValue){
            indexRight = i;
            maxValue = value;
        }
    }

    *currentLeftX = indexLeft;
    *currentRightX = indexRight;
}

void Curves::convertPointToArray(vector<cv::Point> locations, int allPixelsX[], int allPixelsY[]){
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

void Curves::nextY(const int w, int *lowY, int *highY) {
    *lowY = this->h - (w + 1) * this->windowHeight;
    *highY = this->h - w * this->windowHeight;
}

void Curves::nextX(const int current, int *leftX, int *rightX) {
    *leftX = current - this->margin;
    *rightX = current + this->margin;
}

void Curves::nextMidX(const int pixelIndices[], int *currentIndex) {
    int pixelIndicesLength = sizeof(pixelIndices) / sizeof(*pixelIndices);
    if (pixelIndicesLength > this->minPix) {
        int indexSum = 0;
        for (int i = 0; i < pixelIndicesLength; i++) {
            int currentPixelIndex = pixelIndices[i];
            indexSum += this->allPixelsX[currentPixelIndex];
        }
        double meanIndexSum = double(indexSum) / pixelIndicesLength;
        *currentIndex = round(meanIndexSum);
    }

}

void Curves::drawBoundaries(const cv::Point2f p1, const cv::Point2f p2, const cv::Scalar& color, int thickness) {
    cv::rectangle(this->outImg, p1, p2, color, thickness);
}

void Curves::indicesWithinBoundary(const int lowY, const int highY, const int leftX, const int rightX, cv::Mat returnMat) {
    int numberOfPixels = sizeof(this->allPixelsX)/sizeof(*(this->allPixelsX));

    for (int i = 0; i < numberOfPixels; ) {
        if (this->allPixelsX[i] >= leftX && this->allPixelsX[i] <= rightX &&
            this->allPixelsY[i] >= lowY && this->allPixelsY[i] < highY)
            returnMat.at<int>(allPixelsX[i], allPixelsY[i]) = 1;
    }
}

void Curves::pixelLocations(const int indices[], int pixelsX[], int pixelsY[], int length) {
    for (int i = 0; i < length; i++) {
        int index = indices[i];
        pixelsX[i] = this->allPixelsX[index];
        pixelsY[i] = this->allPixelsY[index];
    }
}

void Curves::plot(int t) {

}

void Curves::getRealCurvature(const int xs[], const int ys[], double coefficients[]) {

}

void Curves::radiusOfCurvature(const double y, const double coefficients[], double *radius) {

}

void Curves::updateVehiclePosition(double *vehiclePosition) {

}
