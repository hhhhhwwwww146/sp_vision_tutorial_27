#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tasks/apriltag_detector.hpp" 
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    Camera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml");
    
    auto_charge::AprilTagDetector tag_detector("./configs/yolo.yaml");

    cv::namedWindow("img", cv::WINDOW_NORMAL);

    while (1)
    {
        camera.read(); 
        cv::Mat img = camera.read();

        if (img.empty())
        {
            continue;
        }

        std::list<auto_aim::Armor> armors = yolo.detect(img);
        for (const auto& armor : armors)
        {
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 3);

            std::string color_str = auto_aim::COLORS[armor.color];
            std::string name_str = auto_aim::ARMOR_NAMES[armor.name];
            std::string armor_text = color_str + name_str;

            cv::Point text_pos(armor.points[0].x, armor.points[0].y - 30);
            tools::draw_text(img, armor_text, text_pos, cv::Scalar(0, 255, 0), 2.0, 3);
        }

        std::vector<auto_charge::TagDetection> tags = tag_detector.detect(img);
        for (const auto& tag : tags)
        {
            tools::draw_points(img, tag.corners, cv::Scalar(0, 255, 0), 3);

            std::string tag_text = "36h11-" + std::to_string(tag.id);
            
            cv::Point text_pos(tag.center.x, tag.center.y - 30);
            tools::draw_text(img, tag_text, text_pos, cv::Scalar(0, 255, 0), 2.0, 3);
        }

        cv::imshow("img", img);
        
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}