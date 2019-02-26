/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

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
    int lengthLeft = leftPixelsX.rows;
    int lengthRight = rightPixelsX.rows;

    polyfit(this->leftPixelsY, this->leftPixelsX, this->leftFitCurvePix, 2);
    polyfit(this->rightPixelsY, this->rightPixelsX, this->rightFitCurvePix, 2);

    double kL[3];
    double kR[3];
    for (int i = 0; i < 3; i++) {
        kL[i] = this->leftFitCurvePix.at<int>(0,i);
        kR[i] = this->rightFitCurvePix.at<int>(0,i);
    }

    double ys[this->h];
    for (int i = 0; i < this->h; i++) {
        ys[i] = i * ((this->h - 1) / this->h);
    }

    double leftXs[lengthLeft];
    for (int i = 0; i < lengthLeft; i++) {
        leftXs[i] = kL[0] * pow(ys[i], 2.0) + kL[1] * ys[i] + kL[2];
    }

    double rightXs[lengthRight];
    for (int i = 0; i < lengthRight; i++) {
        rightXs[i] = kR[0] * pow(ys[i], 2.0) + kR[1] * ys[i] + kR[2];
    }

    // TODO: cv::line and stuff
}

void Curves::getRealCurvature(const int xs[], const int ys[], double coefficients[]) {
    for(size_t i = 0; i < sizeof(xs); i++)
    {

    }
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
        kL[i] = this->leftFitCurvePix.at<double>(0,i);
        kR[i] = this->rightFitCurvePix.at<double>(0,i);
    }
    double xL = kL[0] * pow(y, 2) + kL[1] * y + kL[2];
    double xR = kR[0] * pow(y, 2) + kR[1] * y + kR[2];
    double pixPos = xL + (xR - xL) / 2.0;
    this->vehiclePosition = (pixPos - mid) * this->xmPerPixel;
}

CurvesResult Curves::fit(cv::Mat binary) {
    // TODO: Do not forget to convert from array to cv::Mat when getting values back from pixelLocations()
}


void Curves::polyfit(const cv::Mat& srcX, const cv::Mat& srcY, cv::Mat& dst, int order)
{
    cv::Mat X;
    X = cv::Mat::zeros(srcX.rows, order+1, CV_32FC1);
    cv::Mat copy;
    for(int i = 0; i <=order;i++)
    {
        copy = srcX.clone();
        cv::pow(copy,i,copy);
        cv::Mat M1 = X.col(i);
        copy.col(0).copyTo(M1);
    }
    cv::Mat X_t, X_inv;
    cv::transpose(X, X_t);
    cv::Mat temp = X_t*X;
    cv::Mat temp2;
    cv::invert (temp,temp2);
    cv::Mat temp3 = temp2*X_t;
    cv::Mat W = temp3*srcY;
    W.copyTo(dst);
}
