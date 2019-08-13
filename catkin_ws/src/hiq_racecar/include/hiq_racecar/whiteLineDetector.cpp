#include "whiteLineDetector.h"

WhiteLineDetector::WhiteLineDetector() {
    cv::Point2f sourcePoints[4];
    sourcePoints[0] = cv::Point2f( 580, 460 );
    sourcePoints[1] = cv::Point2f( 205, 720);
    sourcePoints[2] = cv::Point2f( 1110 , 720);
    sourcePoints[3] = cv::Point2f( 703 , 460);

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
    curves = Curves(numberOfWindows, margin, minimumPixels, ymPerPixel, xmPerPixel);
}


PipelineOutput WhiteLineDetector::pipeLine(cv_bridge::CvImagePtr imagePtr) {
     cv::Mat imageRaw = imagePtr->image;
     return pipeLineWithImage(imageRaw);
}


PipelineOutput WhiteLineDetector::pipeLineWithImage(cv::Mat source_image) {
      cv::imshow("Source image", source_image);
      cv::waitKey(0);

      cv::Mat emptyImage = cv::Mat::zeros(source_image.size(), source_image.type());

//      cv::Mat undistortedImage;
//      birdsEye.undistort_image(&source_image, &undistortedImage);
//      cv::imshow("Undistorted", undistortedImage);
//      cv::waitKey(0);

      cv::Mat imageBinary = laneFilter.apply(/*undistortedImage*/source_image);
      cv::imshow("Lane filter", imageBinary);
      cv::waitKey(0);

      cv::Mat skyViewImage;
      birdsEye.skyView(imageBinary, &skyViewImage);
      cv::imshow("Sky view", skyViewImage);
      cv::waitKey(0);

      cv::Mat imageWb = skyViewImage & regionOfInterest(imageBinary, 125, 1200);
      cv::imshow("imageWB", imageWb);
      cv::waitKey(0);

      CurvesResult result = curves.fit(imageWb);
      printf("result left radius: %f", result.leftRadius);
      double coeffLeft[3];
      double coeffRight[3];
      for (int i = 0; i < 3; i++) {
          coeffLeft[i] = result.leftFitCurveF.at<double>(0,i);
          coeffRight[i] = result.rightFitCurveF.at<double>(0,i);
      }
      cv::Scalar color = cv::Scalar(0,0,255,255);
      cv::Mat imageGroundWithProjection;
      birdsEye.project(source_image, &imageGroundWithProjection, coeffLeft, coeffRight, &color);
      cv::imshow("imageGroundWithProjection", imageWb);
      cv::waitKey(0);

      stringstream streamPos;
      streamPos << fixed << setprecision(2) << result.vehiclePosition;
      std::string textPos = "vehicle position: " + streamPos.str();
      stringstream streamLeft;
      streamLeft << fixed << setprecision(2) << result.leftRadius;
      std::string textLeft = "left radius: " + streamLeft.str();
      stringstream streamRight;
      streamRight << fixed << setprecision(2) << result.rightRadius;
      std::string textRight = "right radius: " + streamRight.str();

      cv::putText(imageGroundWithProjection, textLeft, cv::Point2f(20,40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255,255));
      cv::putText(imageGroundWithProjection, textRight, cv::Point2f(400,40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255,255));
      cv::putText(imageGroundWithProjection, textPos, cv::Point2f(20,80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255,255));

      PipelineOutput output = {imageGroundWithProjection,
                               result.vehiclePosition,
                               result.leftRadius,
                               result.rightRadius};

      return output;


//      PipelineOutput output = {source_image,
//                                   0,
//                                   0,
//                                   0};
//          return output;
}
