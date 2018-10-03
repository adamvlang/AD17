#include "whiteLineDetector.h"

WhiteLineDetector::WhiteLineDetector() {
    cv::Mat sourcePoints = [180, 180, 0, 360, 640, 340, 460, 180];
    cv::Mat destinationPoints = [160, 0, 160, 360, 480, 360, 480, 0];
    // TODO: Load calibration data with "pickle" and extract calibration matrix and distortion coef 
    //cv::Mat cameraMatrix = ;
    //double distortionCoefficient = ;
    birdsEye = BirdsEye(sourcePoints, destinationPoints, cameraMatrix, distortionCoefficient);

    int saturationThreshold = 120;
    int lightThreshold = 40;
    int lightThresholdArg = 205;
    double gradientThresholdLow = 0.7;
    double gradientThresholdHigh = 1.4;
    int magnitudeThreshold = 40;
    int xThreshold = 20;
    laneFilter = LaneFilter(saturationThreshold, lightThreshold, lightThresholdArg,
                            gradientThresholdLow, gradientThresholdHigh, magnitudeThreshold,
                            xThreshold);

    int numberOfWindows = 1;
    int margin = 100;
    int minimumPixels = 50;
    double ymPerPixel = 30.0 / 720;
    double xmPerPixel = 3.7 / 720;
    curves = Curves(numberOfWindows, margin, minimumPixels, ymPerPixel, xmPerPixel);

    helpers = Helpers();
}

PipeLineOutput WhiteLineDetector::pipeLine(cv_bridge::CvImagePtr image) {
    cv::Mat imageRaw = image->image;
    cv::Mat imageGround = birdsEye.undistort(imageRaw);
    cv::Mat imageBinary = laneFilter.apply(imageGround);
    cv::Mat imageWb = birdsEye.skyView(imageBinary) && helpers->regionOfInterest(imageBinary);
    Curves::CurvesFitOutput result = curves->fit(imageWb);
    cv::Mat imageGroundWithProjection = birdsEye.project(groundImage, imageBinary);

    std::string textPos = "vehicle position: " + result.vehiclePosition;
    stringstream streamLeft;
    streamLeft << fixed << setprecision(2) << result.radiusLeft;
    std::string textLeft = "left radius: " + streamLeft.str();
    stringstream streamRight;
    streamRight << fixed << setprecision(2) << result.radiusLeft;
    std::string textRight = "left radius: " + streamRight.str();

    cv::putText(imageGroundWithProjection, textLeft, Point2f(20,40), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));
    cv::putText(imageGroundWithProjection, textRight, Point2f(400,40), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));
    cv::putText(imageGroundWithProjection, textPos, Point2f(20,80), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));

    PipelineOutput output = {imageGroundWithProjection,
                             result.vehiclePosition,
                             result.radiusLeft,
                             result.radiusRight};

    return output;
}
