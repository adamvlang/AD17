#include "laneFilter.h"
#include "helpers.h"
#include <math.h>

#define PI 3.14159265

LaneFilter::LaneFilter(int saturationThreshold,
                       int lightThreshold,
                       int lightThresholdArg,
                       double gradientThresholdLow,
                       double gradientThresholdHigh,
                       int magnitudeThreshold,
                       int xThreshold) {
    this->saturationThreshold = saturationThreshold;
    this->lightThreshold = lightThreshold;
    this->lightThresholdArg = lightThresholdArg;
    this->gradientThresholdLow = gradientThresholdLow;
    this->gradientThresholdHigh = gradientThresholdHigh;
    this->magnitudeThreshold = magnitudeThreshold;
    this->xThreshold = xThreshold;
}

LaneFilter::LaneFilter() {
};

cv::Mat LaneFilter::sobelBreakdown(cv::Mat image) {
    apply(image);
    cv::Mat b1 = z.clone();
    cv::Mat b2 = z.clone();
    cv::Mat b3 = z.clone();

    b1.setTo(255, sobelCond1);
    b2.setTo(255, sobelCond2);
    b3.setTo(255, sobelCond3);

    cv::Mat returnMat;
    std::vector<cv::Mat> channels;
    channels.push_back(b3);
    channels.push_back(b2);
    channels.push_back(b1);
    merge(channels, returnMat);

    return returnMat;
}

cv::Mat LaneFilter::colorBreakdown(cv::Mat image) {
    apply(image);
    cv::Mat b1 = z.clone();
    cv::Mat b2 = z.clone();

    b1.setTo(255, colorCond1);
    b2.setTo(255, colorCond2);

    cv::Mat returnMat;
    std::vector<cv::Mat> channels;
    channels.push_back(z);
    channels.push_back(b2);
    channels.push_back(b1);
    merge(channels, returnMat);

    return returnMat;
}

cv::Mat LaneFilter::apply(cv::Mat rgbImage) {
    cv::cvtColor(rgbImage, hls, cv::COLOR_RGB2HLS);

    cv::Mat hlsSplit[3];
    split(hls, hlsSplit); // TODO: Does this really split into H-L-S, or is it S-H-L?

    l = hlsSplit[1];
    s = hlsSplit[2];
    z = s.clone();
    z.setTo(cv::Scalar::all(0));
    //z = cv::Mat::all(0); // Found at https://stackoverflow.com/questions/17041758/how-to-fill-matrix-with-zeros-in-opencv
                         // No clue if it works

    cv::Mat colorImage = applyColorMask();
    cv::Mat sobelImage = applySobelMask();

    cv::Mat filteredImage;
    cv::bitwise_or(colorImage, sobelImage, filteredImage);
    return filteredImage;
}

cv::Mat LaneFilter::applyColorMask() {
    // Color cond 1
    cv::Mat satThreshCompMat = s > saturationThreshold;
    cv::Mat lightThreshCompMat = l > lightThreshold;
    cv::bitwise_and(satThreshCompMat, lightThreshCompMat, colorCond1);

    // Color cond 2
    colorCond2 = l > lightThresholdArg;

    // Compute b
    cv::Mat colorCond1or2;
    cv::bitwise_or(colorCond1, colorCond2, colorCond1or2);
    cv::Mat b = z.clone();
    b.setTo(1, colorCond1or2);

    return b;
}

cv::Mat LaneFilter::applySobelMask() {
    cv::Mat lx;
    cv::Mat ly;
    // cv::CV_64F = 6
    cv::Sobel(l, lx, 6, 1, 0, 5);
    cv::Sobel(l, ly, 6, 0, 1, 5);

    // Temp clone lx and then compute the real values
    cv::Mat gradL = lx.clone();
    cv::Mat magL = lx.clone();
    for (int col = 0; col < gradL.cols; col++) {
        for (int row = 0; row < gradL.rows; row++) {
            gradL.at<double>(row, col) = atan2(ly.at<double>(row, col), lx.at<double>(row, col)) * 180 / PI;
            magL.at<double>(row, col) = pow(lx.at<double>(row, col), 2) + pow(ly.at<double>(row, col), 2);
        }
    }

    cv::Mat slm = scaleAbs(magL);
    cv::Mat slx = scaleAbs(lx);

    cv::Mat b = z.clone();
    sobelCond1 = slm > magnitudeThreshold;
    sobelCond2 = slx > xThreshold;
    cv::bitwise_and(gradL > gradientThresholdLow, gradL < gradientThresholdHigh, sobelCond3);

    cv::Mat sobelCond1and2;
    cv::Mat sobelCond1and2and3;
    cv::bitwise_and(sobelCond1, sobelCond2, sobelCond1and2);
    cv::bitwise_and(sobelCond1and2, sobelCond3, sobelCond1and2and3);
    b.setTo(1, sobelCond1and2and3);
    // TODO: Continue here
}
