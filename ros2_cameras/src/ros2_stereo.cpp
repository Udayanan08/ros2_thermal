#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.



#include <opencv2/core.hpp>


#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <unistd.h>
#include <stdio.h>

using namespace std;



 
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher(cv::Mat fr, cv::VideoCapture capt) : Node("stereo_image_publisher"), frame(fr), cap(capt)
   {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("stereo_left", 10);
    publisher2_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_right", 10);
    timer_ = this->create_wall_timer(
        33ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
 
private:
  cv::Mat frame;
  cv::VideoCapture cap;
  
  
  void timer_callback() {

    // Create a new 640x480 image
    cap.read(frame);

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame(cv::Range(0, frame.size().height), cv::Range(0, frame.size().width/2)))
               .toImageMsg();
    msg2_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame(cv::Range(0, frame.size().height), cv::Range(frame.size().width/2, frame.size().width)))
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    publisher2_->publish(*msg2_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  sensor_msgs::msg::Image::SharedPtr msg2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher2_;
  size_t count_;

};
 
int main(int argc, char *argv[]) {
  
  cv::Mat cv_frame;
  
  cv::VideoCapture cap;
  cap.open(2);
  cap.set(cv::CAP_PROP_FPS,30);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>(cv_frame, cap);
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
