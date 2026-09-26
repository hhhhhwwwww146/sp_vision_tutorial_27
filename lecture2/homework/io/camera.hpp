#pragma once

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "MvCameraControl.h"

class Camera {
public:
    Camera();
    ~Camera();
    cv::Mat read();

private:
    cv::Mat transfer(MV_FRAME_OUT &raw);
    
    void* handle_; 
};