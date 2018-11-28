#include "whiteLineDetector.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudaarithm.hpp"

int main() {
    cv::Mat warpedImage = cv::imread("/home/polestar/", CV_LOAD_IMAGE_COLOR);

    WhiteLineDetector whiteLineDetector = WhiteLineDetector();

    PipelineOutput laneImageOutput = whiteLineDetector.pipeLineWithImage(warpedImage);

    cv::imshow("Test", laneImageOutput.image);
    cv::waitKey(0);

    return 0;
}
