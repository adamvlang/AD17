#include "BirdsEye.h"
#include <boost/format.hpp>

using namespace cv;

BirdsEye::BirdsEye(Point2f sourcepoints_l[], Point2f destpoints_l[], Mat camMatrix, double distCoef)
{
    for (int i = 0; i < 4; i++) {
        this->srcPoints[i] = sourcepoints_l[i];
        this->destPoints[i] = destpoints_l[i];
    }

    this->camMatrix = camMatrix;
    this->distCoef = distCoef;

    this->warpMatrix = getPerspectiveTransform(this->srcPoints, this->destPoints);
    this->invWarpMatrix = getPerspectiveTransform(this->srcPoints, this->destPoints);
}

BirdsEye::~BirdsEye(){
}

void BirdsEye::undistort_image(Mat* rawImage, Mat* undistoredImage){

    undistort(*rawImage, *undistoredImage, camMatrix, distCoef);
}

void BirdsEye::skyView(const Mat groundImage, Mat* outputImage){

    //TODO(albin): uncomment this when we have a functional callibration matrix
    //Mat undistorted_image;
    //undistort(&ground_image, undistored_image);
    Size shape= groundImage.size();
    warpPerspective(groundImage, *outputImage, warpMatrix, shape);
}

void BirdsEye::project(Mat groundImage, Mat* outputImage, double *leftFit, double *rightFit, Scalar *color){
    /*
     Function which projects the ground image to a sky image.
     */
    cv::Mat skyLane = Mat::zeros(groundImage.size(), groundImage.type());

    int imageHeight = skyLane.size().height;
    int maxHeight = skyLane.size().height;

    //Linspace :D
    double ys[imageHeight];
    double diff = 0;
    for (int i = 0; i < imageHeight; i++) {
        diff = diff + (maxHeight/imageHeight);
        ys[i] = diff;
    }

    //Quadratic funcition
    double lxs[imageHeight];
    double rxs[imageHeight];
    for (int i = 0; i < imageHeight; i++) {
        lxs[i] = leftFit[0] * ys[i]*ys[i] + leftFit[1] * ys[i] + leftFit[2];
        rxs[i] = rightFit[0] * ys[i]*ys[i] + rightFit[1] * ys[i] + rightFit[2];
    }

    //Convert the polygon points_l to s points_l
    Point pointsL[1][2*imageHeight];

    for (int i = 0; i < imageHeight; i++) {
        std::cout << boost::format("values: %s %s %s \n") % ys[i] % lxs[i] % rxs[i];
        pointsL[0][i]= Point(ys[i], lxs[i]);
    }

    for (int i = 0; i < imageHeight; i++) {
    //    std::cout << boost::format("values: %s %s %s \n") % ys[i] % lxs[i] % rxs[i];
        pointsL[0][i + imageHeight]= Point(ys[i], rxs[i]);
    }

    const Point* ppt[1] = { pointsL[0]};

    int npt[] = { 2*imageHeight };

    fillPoly(skyLane, ppt, npt, 1, Scalar( 0, 0, 255), 1);
    Size shape = skyLane.size();
    Mat lambda = Mat::zeros(skyLane.rows, skyLane.cols, skyLane.type() );
    lambda = getPerspectiveTransform(this->srcPoints, this->destPoints);
    Mat warp_image;

    warpPerspective(skyLane, warp_image, lambda, shape);
    Mat result = Mat::zeros(skyLane.rows, skyLane.cols, groundImage.type());
    addWeighted(groundImage, 1, warp_image, 0.3, 0, *outputImage);
}


