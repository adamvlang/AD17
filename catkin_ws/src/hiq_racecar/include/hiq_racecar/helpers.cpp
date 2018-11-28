#include "helpers.h"

cv::Mat regionOfInterest(cv::Mat imageGray, int mn = 125, int mx = 1200) {
	cv::Mat m(imageGray);
	m += cv::Scalar(1);
    m = m(cv::Range::all(), cv::Range(mn,mx));
	return m;
}

/* The original python function
def roi(gray, mn = 125, mx = 1200):
  m = np.copy(gray) + 1
  m[:, :mn] = 0 
  m[:, mx:] = 0 
  return m 
*/

cv::Mat scaleAbs(cv::Mat x, int m) {

    double maxValue = 0;
    for (int col = 0; col < x.cols; col++) {
        for (int row = 0; row < x.rows; row++) {
            double currentValue = x.at<double>(row, col);
            if (abs(currentValue) > abs(maxValue)) {
                maxValue = abs(currentValue);
            }
        }
    }

    x /= maxValue;

    cv::Mat result;
    cv::convertScaleAbs(x, result, m);

    return result;
}
