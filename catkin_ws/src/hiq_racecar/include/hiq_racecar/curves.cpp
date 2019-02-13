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
    int lengthLeft = sizeof(this->leftPixelsX) / sizeof(this->leftPixelsX[0]);
    int lengthRight = sizeof(this->rightPixelsX) / sizeof(this->rightPixelsX[0]);

    polyfit(this->leftPixelsY, this->leftPixelsX, this->leftFitCurvePix, 2);
    polyfit(this->rightPixelsY, this->rightPixelsX, this->rightFitCurvePix, 2);

    double kL[3];
    double kR[3];
    for (int i = 0; i < 3; i++) {
        kL[i] = this->leftFitCurvePix[i];
        kR[i] = this->rightFitCurvePix[i];
    }

    double ys[this->h];
    for (int i = 0; i < this->h; i++) {
        ys[i] = i * ((this->h - 1) / this->h);
    }

    double leftXs = kL[0] * pow(ys, 2) + kL[1] * ys + kL[2];
    double rightXs = kR[0] * pow(ys, 2) + kR[1] * ys + kR[2];
}

void Curves::getRealCurvature(const int xs[], const int ys[], double coefficients[]) {

}

void Curves::radiusOfCurvature(const double y, const double coefficients[], double *radius) {
    double f1Squared = pow(coefficients[1], 2);
    double numinator = 1 + pow(2 * coefficients[0] * y + f1Squared, 1.5);
    double denominator = fabs(2 * coefficients[0]);
    *radius = numinator / denominator;
}

void Curves::updateVehiclePosition() {
    int y = this->h;
    int mid = (int) this->w / 2;
    double kL[3];
    double kR[3];
    for (int i = 0; i < 3; i++) {
        kL[i] = this->leftFitCurvePix[i];
        kR[i] = this->rightFitCurvePix[i];
    }
    double xL = kL[0] * pow(y, 2) + kL[1] * y + kL[2];
    double xR = kR[0] * pow(y, 2) + kR[1] * y + kR[2];
    double pixPos = xL + (xR - xL) / 2.0;
    this->vehiclePosition = (pixPos - mid) * this->xmPerPixel;
}

CurvesResult Curves::fit(cv::Mat binary) {

}
