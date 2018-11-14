//
//  main.cpp
//  WhiteLineDetection
//
//  Created by Albin Lorentzson on 2018-09-22.
//  Copyright © 2018 Albin Lorentzson. All rights reserved.
//

#include <iostream>
#include "BirdsEye.h"

using namespace cv;

int main(int argc, const char * argv[]) {
    Mat source_image = imread("/home/alorent4/NonWorkProjects/WhiteLineDetection/test1.jpg", CV_LOAD_IMAGE_COLOR);
    String title = "Title";
    //namedWindow( title, WINDOW_AUTOSIZE );
    //imshow(title, source_image);
    //waitKey(0);

    Mat output;
    //Initialize some variables
    Point2f inputQuad[4];
    inputQuad[0] = Point2f( -30,-60 );
    inputQuad[1] = Point2f( source_image.cols+50,-50);
    inputQuad[2] = Point2f( source_image.cols+100,source_image.rows+50);
    inputQuad[3] = Point2f( -50,source_image.rows+50  );

    //Initialize some more
    Point2f outputQuad[4];
    outputQuad[0] = Point2f( 0,0 );
    outputQuad[1] = Point2f( source_image.cols-1, 0);
    outputQuad[2] = Point2f( source_image.cols-1, source_image.rows-1);
    outputQuad[3] = Point2f( 0,source_image.rows-1);

    BirdsEye birdsEye = BirdsEye(inputQuad, outputQuad, source_image, 0.2);

    Mat outputImage = birdsEye.skyView(source_image);
    //imshow("Skyview", outputImage);
    //waitKey(0);

    //From curves we should get some best fit quadratic function
    double best_fit_left[3] = {0.001, 0.3, 200};
    double best_fit_right[3] = {0.0009, -0.7, 600};
    cv::Scalar color = cv::Scalar(100,100,100);



    //Mat temp_image = Mat::zeros(source_image.size(), CV_8UC3 );
    Mat polygon_image = birdsEye.project(source_image, source_image,
                                        best_fit_left, best_fit_right, &color);
    imshow("Polygon", polygon_image);
    waitKey(0);
}
