#include "helpers.h"

cv::Mat Helpers::regionOfInterest(cv::Mat imageGray, int mn = 125, int mx = 1200) {
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
