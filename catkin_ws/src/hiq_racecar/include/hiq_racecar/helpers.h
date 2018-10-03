#ifndef _HELPERS_H
#define _HELPERS_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

class Helpers {
    public:
        PipeLineOutput regionOfInterest(cv::Mat imageBinary);

}

#endif _HELPERS_H
