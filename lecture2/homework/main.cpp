#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include <iostream>

int main()
{
    Camera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml"); 

    while (1) 
    {
        cv::Mat img = camera.read();
        if (img.empty()) 
        {
            continue; 
        }

        std::list<auto_aim::Armor> armors = yolo.detect(img);

        for (const auto& armor : armors) 
        {
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
        }

        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        
        if (cv::waitKey(1) == 'q') 
        {
            break;
        }
    }
    
    return 0;
}