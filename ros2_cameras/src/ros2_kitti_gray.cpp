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
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <vector>

using namespace std;
 
using namespace std::chrono_literals;


 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher(cv::String dir_number) : Node("stereo_image_publisher"), dir_num(dir_number)
   {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("stereo_left", 10);
    publisher2_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_right", 10);
    timer_ = this->create_wall_timer(
        33ms, std::bind(&MinimalImagePublisher::timer_callback, this));
    cv::utils::fs::glob(dir,"*.png", dir_vec,false,false);
    cv::utils::fs::glob(dir1,"*.png", dir_vec1,false,false);
    cout<<dir_vec.size()<<endl;
  }
  

  
 
private:
  void timer_callback() {
    if(count_<dir_vec.size()){
      // Create a new 640x480 image
      img0 = cv::imread(dir_vec[count_], cv::IMREAD_UNCHANGED);
      img1 = cv::imread(dir_vec1[count_], cv::IMREAD_UNCHANGED);

      // Write message to be sent. Member function toImageMsg() converts a CvImage
      // into a ROS image message
      msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img0).toImageMsg();
      msg2_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img1).toImageMsg();
  
      // Publish the image to the topic defined in the publisher
      publisher_->publish(*msg_.get());
      publisher2_->publish(*msg2_.get());
      RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
      count_++;
    }else{
      RCLCPP_INFO(this->get_logger(), "All images are published, Shutting down now!!!");
      rclcpp::shutdown();
    } 
    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  sensor_msgs::msg::Image::SharedPtr msg2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher2_;
  size_t count_=0;
  //cv::Mat frame;
  //cv::VideoCapture cap;
  cv::String dir_num;
  const cv::String dir = "/home/drdo/c_codes/datasets/data_odometry_gray/dataset/sequences/"+dir_num+"/image_0/";
  const cv::String dir1 = "/home/drdo/c_codes/datasets/data_odometry_gray/dataset/sequences/"+dir_num+"/image_1/";
  vector<cv::String> dir_vec;
  vector<cv::String> dir_vec1;
  cv::Mat img0;
  cv::Mat img1;
  unsigned int i;
};
 
int main(int argc, char *argv[]) {
  
  // cv::Mat cv_frame;
  
  // cv::VideoCapture cap;
  // cap.open(4);
  // cap.set(cv::CAP_PROP_FPS,30);
  // cap.set(cv::CAP_PROP_FRAME_WIDTH, 360);
  // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  cv::String dir_number = argv[1];
  // cout<<dir_number<<endl;

  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>(dir_number);
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
