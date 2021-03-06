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

Curves::Curves(int numberOfWindows, int margin, int minPix, double ymPerPixel, double xmPerPixel) {
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

    convertPointToArray(locations);
}

void Curves::start(const cv::Mat binary, int *currentLeftX, int *currentRightX){
    cv::Mat bottom_half = binary(cv::Range((int)(this->h/2), this->h), cv::Range(0, this->w));
    vector<int> hist;
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM); // TODO: Use CalcHist? No!
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

void Curves::convertPointToArray(vector<cv::Point> locations){
    int count = locations.size();
    this->allPixelsY.clear();
    this->allPixelsX.clear();
    vector<int> allPixelsX(count, 0);
    vector<int> allPixelsY(count, 0);
    for(int i = 0; i < count; i++) {
        cv::Point point = locations[i];
        allPixelsX.at(i) = point.x;
        allPixelsY.at(i) = point.y;
    }
}

void Curves::nextY(const int w, int *lowY, int *highY) {
    *lowY = this->h - (w + 1) * this->windowHeight;
    *highY = this->h - w * this->windowHeight;
}

void Curves::nextX(const int current, int *leftX, int *rightX) {
    printf("current is %i\n", current);
    printf("leftX is %i\n", *leftX);
    printf("rightX is %i\n", *rightX);
    printf("margin is %i\n", this->margin);
    *rightX = 369 + 100;//current + this->margin;
    printf("I calculated rightX\n");
    *leftX = current - this->margin;
}

void Curves::nextMidX(const int pixelIndices[], int *currentIndex) {
    int pixelIndicesLength = sizeof(pixelIndices) / sizeof(*pixelIndices);
    if (pixelIndicesLength > this->minPix) {
        int indexSum = 0;
        for (int i = 0; i < pixelIndicesLength; i++) {
            int currentPixelIndex = pixelIndices[i];
            indexSum += this->allPixelsX.at(currentPixelIndex);
        }
        double meanIndexSum = double(indexSum) / pixelIndicesLength;
        *currentIndex = round(meanIndexSum);
    }

}

void Curves::drawBoundaries(const cv::Point2f p1, const cv::Point2f p2, const cv::Scalar& color, int thickness) {
    cv::rectangle(this->outImg, p1, p2, color, thickness);
}

void Curves::indicesWithinBoundary(const int lowY, const int highY, const int leftX, const int rightX, vector<int> returnVector) {
    int numberOfPixels = allPixelsX.size();
    for (int i = 0; i < numberOfPixels; ) {
        if (this->allPixelsX.at(i) >= leftX && this->allPixelsX.at(i) <= rightX &&
            this->allPixelsY.at(i) >= lowY && this->allPixelsY.at(i) < highY){
            returnVector.push_back(allPixelsX.at(i));
        }
    }
}

void Curves::pixelLocations(const vector<int> indices, cv::Mat pixelsX, cv::Mat pixelsY, int length) {
    for (int i = 0; i < length; i++) {
        int index = indices[i];
        pixelsX.at<int>(0,i) = this->allPixelsX.at(index);
        pixelsY.at<int>(0,i) = this->allPixelsY.at(index);
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

    double xls[lengthLeft];
    for (int i = 0; i < lengthLeft; i++) {
        double leftXs = kL[0] * pow(ys[i], 2.0) + kL[1] * ys[i] + kL[2];
        xls[i] = round(leftXs);
    }

    double xrs[lengthRight];
    for (int i = 0; i < lengthRight; i++) {
        double rightXs = kR[0] * pow(ys[i], 2.0) + kR[1] * ys[i] + kR[2];
        xrs[i] = round(rightXs);
    }

    int stopIndex = min(lengthLeft, lengthRight);
    stopIndex = min(stopIndex, this->h);
    for (int i = 0; i < stopIndex; i++) {
        int xl = xls[i];
        int xr = xrs[i];
        int y = ys[i];
        cv::line(this->outImg, cv::Point(xl - t, y), cv::Point(xl + t, y), cv::Scalar(255, 255, 0), int(t/2));
        cv::line(this->outImg, cv::Point(xr - t, y), cv::Point(xr + t, y), cv::Scalar(0, 0, 255), int(t/2));
    }
}

void Curves::getRealCurvature(const cv::Mat xs, const cv::Mat ys, cv::Mat coefficients) {
    polyfit(xs, ys, coefficients, 2);
}

void Curves::radiusOfCurvature(const double y, const cv::Mat coefficients, double *radius) {
    double f1Squared = pow(coefficients.at<double>(0,1), 2);
    double numinator = 1 + pow(2 * coefficients.at<double>(0,0) * y + f1Squared, 1.5);
    double denominator = fabs(2 * coefficients.at<double>(0,0));
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
    printf("Nu kör vi!\n");
    // TODO: Do not forget to convert from array to cv::Mat when getting values back from pixelLocations()
    storeDetails(binary);
    int *mid_left_x, *mid_right_x;
    start(binary, mid_left_x, mid_right_x);
    printf("I started and mid_left_x is %i\n", *mid_left_x);
    int *y_low, *y_high;
    int *x_left_low, *x_left_high;
    int *x_right_low, *x_right_high;
    *y_low = 1;
    *y_high = 1;
    *x_left_low = 1;
    vector<int> left_pixels_indicies;
    vector<int> right_pixels_indicies;
    printf("I initialized\n");

    for(size_t i = 0; i < this->numberOfWindows; i++)
    {
        nextY(i, y_low, y_high);
        printf("I calculated nextY\n");
        nextX(*mid_left_x, x_left_low, x_left_high);
        printf("I calculated nextX 1\n");
        nextX(*mid_right_x , x_right_low, x_right_high);
        printf("I calculated nextX 2\n");
        
        cv::Point2f corner1 = cv::Point2f(*x_left_low, *y_low);
        cv::Point2f corner2 = cv::Point2f(*x_left_high, *y_high);

        cv::Point2f corner3 = cv::Point2f(*x_right_low, *y_low);
        cv::Point2f corner4 = cv::Point2f(*x_right_high, *y_high);
        
        drawBoundaries(corner1, corner2, cv::Scalar(255,0,0));
        drawBoundaries(corner3, corner4, cv::Scalar(0,255,0));
        printf("I drew boundaries\n");
        
        vector<int> current_left_pixels_indicies;
        vector<int> current_right_pixels_indicies;
        indicesWithinBoundary(*y_low, *y_high, *x_left_low, *x_left_high, current_left_pixels_indicies);
        indicesWithinBoundary(*y_low, *y_high, *x_left_low, *x_left_high, current_right_pixels_indicies);
        printf("I got indices within boundary\n");

        left_pixels_indicies.insert(left_pixels_indicies.end(), current_left_pixels_indicies.begin(), current_left_pixels_indicies.end());
        right_pixels_indicies.insert(right_pixels_indicies.end(), current_right_pixels_indicies.begin(), current_right_pixels_indicies.end());

        int* current_left_pixels_array = &current_left_pixels_indicies[0];
        int* current_right_pixels_array = &current_right_pixels_indicies[0];
        nextMidX(current_left_pixels_array, mid_left_x);
        nextMidX(current_right_pixels_array, mid_right_x);
    }

    pixelLocations(left_pixels_indicies, this->leftPixelsX, this->leftPixelsY, (int) left_pixels_indicies.size());
    pixelLocations(right_pixels_indicies, this->rightPixelsX, this->rightPixelsY, (int) right_pixels_indicies.size());

    getRealCurvature(this->leftPixelsX, this->leftPixelsY, this->leftFitCurveF);
    getRealCurvature(this->rightPixelsX, this->rightPixelsY, this->rightFitCurveF);

    radiusOfCurvature(this->h * this->ymPerPixel, this->leftFitCurveF, &(this->leftRadius));
    radiusOfCurvature(this->h * this->ymPerPixel, this->rightFitCurveF, &(this->rightRadius));

    plot();
    updateVehiclePosition();

    CurvesResult curvesResult = {this->outImg,
                                 this->leftRadius,
                                 this->rightRadius,
                                 this->leftFitCurveF,
                                 this->rightFitCurveF,
                                 this->leftFitCurvePix,
                                 this->rightFitCurvePix,
                                 this->vehiclePosition};

    return curvesResult;
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
