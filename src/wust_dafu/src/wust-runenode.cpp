#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include "wust-runedetector.cpp"

class RMRuneDetectorNode {
public:
    RMRuneDetectorNode() {
        ros::NodeHandle nh;
        pub_ = nh.advertise<geometry_msgs::Point>("target_position", 10);
    }

    void ImageCallback(const cv::Mat& image) {
        if (detector_.lose_target_count > 50) {
            detector_.Reset();
        }
        detector_.PreprocessImage(image);
        detector_.FindLights();
        detector_.DistinguishingLights();
        
        cv::Point2f pos=detector_.DrawRunes();
        geometry_msgs::Point msg;
        msg.x = pos.x;
        msg.y = pos.y;
      
        
        pub_.publish(msg);
    }

private:
    RuneDetector detector_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_position_publisher");
    std::cout << "OpenCV built with FFmpeg: " << cv::getBuildInformation() << std::endl;


    cv::VideoCapture cap("/home/zhang/dafu_final/src/wust_dafu/src/3.mp4"); 

    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file");
        return -1;
    }

    RMRuneDetectorNode node;
    cv::Mat frame;
    int frame_count = 0;

    while (cap.read(frame) && frame_count < 700) { // Limit frame count to 700
        frame_count++;

        int key = cv::waitKey(1);
        if (key == 27) { // ESC to quit
            break;
        } else if (key == 32) { // Space to pause
            cv::waitKey(0); // Wait for any key
        }

        node.ImageCallback(frame);
        ros::spinOnce();
    }

    cap.release();
    return 0;
}
