#include "whiteLineDetector.h"

WhiteLineDetector::WhiteLineDetector() {
    cv::Point2f sourcePoints[4];
    sourcePoints[0] = cv::Point2f( 360, 450 );
    sourcePoints[1] = cv::Point2f(50, 700);
    sourcePoints[2] = cv::Point2f( 1200 , 700);
    sourcePoints[3] = cv::Point2f( 950 , 450);

    cv::Point2f destinationPoints[4];
    destinationPoints[0] = cv::Point2f( 320, 0);
    destinationPoints[1] = cv::Point2f( 320 , 720);
    destinationPoints[2] = cv::Point2f( 960, 720);
    destinationPoints[3] = cv::Point2f( 960, 0);

    // TODO: Load calibration data with "pickle" and extract calibration matrix and distortion coef 
    cv::Mat cameraMatrix = cv::Mat();
    double distortionCoefficient = 0.1;
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
    //TODO: curves = TODO: Curves(numberOfWindows, margin, minimumPixels, ymPerPixel, xmPerPixel);
}


PipelineOutput WhiteLineDetector::pipeLine(cv::Mat source_image) {
//    TODO
//    cv::Mat imageRaw = image->image;
      cv::Mat skyViewImage = cv::Mat::zeros(source_image.size(), source_image.type());
      //birdsEye.undistort_image(&source_image, &undistortedImage);
      //cv::imshow("Undistorted", undistortedImage);
      //cv::waitKey(0);
      cv::imshow("Source image", source_image);
      cv::waitKey(0);
      cv::Mat imageBinary = laneFilter.apply(source_image);
      cv::imshow("Lane filter", imageBinary);
      cv::waitKey(0);
      birdsEye.skyView(imageBinary, &skyViewImage);
      cv::imshow("Sky view", skyViewImage);
      cv::waitKey(0);
      cv::Mat imageWb = skyViewImage & regionOfInterest(imageBinary, 125, 1200);
      cv::imshow("imageWB", imageWb);
      cv::waitKey(0);
//    Curves::CurvesFitOutput result = curves->fit(imageWb);
//    cv::Mat imageGroundWithProjection = birdsEye.project(groundImage, imageBinary);

//    std::string textPos = "vehicle position: " + result.vehiclePosition;
//    stringstream streamLeft;
//    streamLeft << fixed << setprecision(2) << result.radiusLeft;
//    std::string textLeft = "left radius: " + streamLeft.str();
//    stringstream streamRight;
//    streamRight << fixed << setprecision(2) << result.radiusLeft;
//    std::string textRight = "left radius: " + streamRight.str();

//    cv::putText(imageGroundWithProjection, textLeft, cv::Point2f(20,40), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));
//    cv::putText(imageGroundWithProjection, textRight, cv::Point2f(400,40), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));
//    cv::putText(imageGroundWithProjection, textPos, cv::Point2f(20,80), FONT_HERSHEY_COMPLEX, 1,  Scalar(0,0,255,255));

//    PipelineOutput output = {imageGroundWithProjection,
//                             result.vehiclePosition,
//                             result.radiusLeft,
//                             result.radiusRight};

//    return output;


PipelineOutput output = {source_image,
                             0,
                             0,
                             0};
    return output;
}
