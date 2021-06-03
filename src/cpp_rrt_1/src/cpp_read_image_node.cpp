#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class MapImagePublisher : public rclcpp::Node
{
  public: 
    MapImagePublisher(cv::Mat image)
    : Node("map_image_publisher")
    {

      img_ = image;

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("map_image", 1);
      timer_ = this->create_wall_timer(1s, std::bind(&MapImagePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "map";

      cv_bridge::CvImage img_bridge;
      sensor_msgs::msg::Image img_msg;

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img_);
      img_bridge.toImageMsg(img_msg);

      RCLCPP_INFO(this->get_logger(), "Publishing image.");
      publisher_->publish(img_msg);
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::Mat img_;
};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);

  if(argc != 2){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: read_map_image <absolute path to map>");
      return 1;
  }

  cv::Mat img = cv::imread(argv[1], cv::ImreadModes::IMREAD_COLOR);
  
  if(!img.data)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error reading image.");
    return 1;
  }

  if(img.channels() == 3)
  {
  //Convert To Gray Scale
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  }

  //Debug 1 #See if read image is successful
  //bool writeStatus = cv::imwrite("/home/administrator/Downloads/map1.png", img);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Write Status: %d", writeStatus);

  auto img_publisher = std::make_shared<MapImagePublisher>(img);
  rclcpp::spin(img_publisher);
  rclcpp::shutdown();
  return 0;
}