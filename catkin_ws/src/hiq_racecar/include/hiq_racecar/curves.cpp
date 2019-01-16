#include "curves.h"

Curves::Curves() {

    void Curves::storeDetails(const cv::Mat binary){     
        cv::Mat outImg;
        std::vector<cv::Mat> channels;
        channels.push_back(binary);
        channels.push_back(binary);
        channels.push_back(binary);
        merge(channels, outImg*255);
        this->OutImg = outImg;
        this->binary = binary;
        this->h = binary.size().height;
        this->w = binary.size().width;
        this->mid = static_cast<int>(this->h / 2);
        this->windowHeight = static_cast<int>(this->h / this->n);
        vector<cv::Point> locations;
        cv::findNonZero(binary, locations);
        
        covertPointToArray(locations, this->allPixelsX, this->allPixelsY);
    }
    
    void Curves::covertPointToArray(vector<Point> locations, int[] allPixelsX, int[] allPixelsY){
        int count = locations.size;
        int pixelsX[count] = {0};
        int pixelsY[count] = {0};
        for(int i = 0; i < count; i++) {
            cv::Point point = locations[i];
            pixelsX[i] = point.x;
            pixelsY[i] = point.y;
        }

        allPixelsX = pixelsX;
        allPixelsY = pixelsY;
    }
}


