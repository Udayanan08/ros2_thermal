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
#include <fcntl.h>
#include <asm/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

using namespace std;

int width = 640;
int height = 512;
int fd;
char video[20] = "/dev/video0";
int type;


 
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher(cv::Mat fr, cv::Mat fr2, int f, struct v4l2_buffer bi) : Node("ros2_thermal"), frame(fr), frame2(fr2), fd(f), bufferinfo(bi)
   {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("flir_thermal", 10);
    timer_ = this->create_wall_timer(
        33ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
 
private:
  cv::Mat frame;
  cv::Mat frame2;
  int fd=1;
  struct v4l2_buffer bufferinfo;
  
  void timer_callback() {

    // Create a new 640x480 image
    ioctl(fd, VIDIOC_QBUF, &bufferinfo);
    ioctl(fd, VIDIOC_DQBUF, &bufferinfo);
    AGC_Basic_Linear(frame, frame2, 512, 640);

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame2)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  
  void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width) {
	int i, j;  // aux variables

	// auxiliary variables for AGC calcultion
	unsigned int max1=0;         // 16 bits
	unsigned int min1=0xFFFF;    // 16 bits
	unsigned int value1, value2, value3, value4;

	// RUN a super basic AGC
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= min1 ) {
				min1 = value3;
			}
			if ( value3 >= max1 ) {
				max1 = value3;
			}
		}
	}


	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			// printf("%04X \n", value4);

			output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
		}
	}

}
};
 
int main(int argc, char *argv[]) {
  struct v4l2_capability cap;
  // We open the Video Device
  fd = open(video, O_RDWR);
  ioctl(fd, VIDIOC_QUERYCAP, &cap);
  
  //v4l2 format variables
  struct v4l2_format format;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = width;
  format.fmt.pix.height = height;
  
  //requesting desired format
  ioctl(fd, VIDIOC_S_FMT, &format);
  
  //Buffer requesting
  struct v4l2_requestbuffers bufrequest;
  
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 1;   // we are asking for one buffer
  
  //buffer request
  ioctl(fd, VIDIOC_REQBUFS, &bufrequest);
  
  struct v4l2_buffer bufferinfo;
  
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;
  
  //buffer querying
  ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo);
  void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);
  
  cout <<buffer_start << endl;
  memset(buffer_start, 0, bufferinfo.length);
  
  type = bufferinfo.type;
  ioctl(fd, VIDIOC_STREAMON, &type);
  
  cv::Mat frame(height, width, CV_16U, buffer_start);
  cv::Mat frame2(height, width, CV_8U, 1);

  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>(frame, frame2, fd, bufferinfo);
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
