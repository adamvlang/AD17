#ifdef _BIRDS_EYE_H
#define _BIRDS_EYE_H
#endif

#include "opencv2/opencv.hpp"
#include "opencv2/core/mat.inl.hpp"


class BirdsEye
{
public:

    BirdsEye(cv::Point2f sourcePoints[], cv::Point2f destPoints[], cv::Mat, double);
	BirdsEye();
    ~BirdsEye();
	cv::Point2f srcPoints[4];
	cv::Point2f destPoints[4];
	cv::Mat camMatrix;
	cv::Mat warpMatrix;
	cv::Mat invWarpMatrix;
    double distCoef;

	void undistort_image(cv::Mat* raw_image, cv::Mat* undistorted_image);

	void skyView(const cv::Mat ground_image, cv::Mat* output_image);

    void project(cv::Mat ground_image, cv::Mat* output_image, double left_fit[3], double right_fit[3], cv::Scalar *color);
};
