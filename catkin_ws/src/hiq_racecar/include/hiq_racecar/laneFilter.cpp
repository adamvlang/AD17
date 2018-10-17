#include "laneFilter.h"

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

// TODO: Continue here!!
cv::Mat LaneFilter::applyColorMask() {

}

cv::Mat LaneFilter::applySobelMask() {

}
