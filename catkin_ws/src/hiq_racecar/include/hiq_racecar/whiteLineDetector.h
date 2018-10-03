#ifndef _WHITE_LINE_DETECTOR_H
#define _WHITE_LINE_DETECTOR_H

#include <string>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudaarithm.hpp"

#include <cv_bridge/cv_bridge.h>

#include "birdsEye.h"
#include "laneFilter.h"
#include "curves.h"
#include "helpers.h"

struct PipelineOutput {
    cv_bridge::CvImagePtr image;
    double position;
    double radiusLeft;
    double radiusRight;
};

class WhiteLineDetector {
    public:
        PipeLineOutput pipeLine(cv_bridge::CvImagePtr image);
    private:
        BirdsEye birdsEye;
        LaneFilter laneFilter;
        Curves curves;
        Helpers helpers;
}

#endif _WHITE_LINE_DETECTOR_H
