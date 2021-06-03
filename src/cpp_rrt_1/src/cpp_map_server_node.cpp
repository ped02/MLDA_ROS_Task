#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::placeholders;

class MapServer : public rclcpp::Node
{
  public: 
    MapServer()
    : Node("map_server")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("map_image", 1, std::bind(&MapServer::construct_map_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Starting Map Server");
    }

  private:
    void construct_map_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Receiving Map Image");
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = img_msg->header.frame_id;


      cv_bridge::CvImagePtr img_bridge_ptr;

      img_bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC1);

      cv::Mat img;
      img = img_bridge_ptr->image;

      std::vector<int8_t> occupancy_grid_data;
      
      RCLCPP_INFO(this->get_logger(), "Image Converted %d x %d", img_msg->width, img_msg->height);

      {
        cv::MatIterator_<uchar> it, end;
        int8_t d;

        RCLCPP_INFO(this->get_logger(), "Set Loop");

        for(it = img.begin<uchar>(), end = img.end<uchar>(); it != end; ++it)
        {
          //RCLCPP_INFO(this->get_logger(), "Starting Loop");
          //RCLCPP_INFO(this->get_logger(), "Starting Loop pixel value: %d", *it);
          d = 0;
          if(*it != 0)
          {
            d = 100;
          }
          occupancy_grid_data.push_back(d);
          
        } 

      }
      RCLCPP_INFO(this->get_logger(), "Map Converted");

      nav_msgs::msg::OccupancyGrid grid_msg;

      nav_msgs::msg::MapMetaData map_meta;

      map_meta.map_load_time = img_msg->header.stamp;
      map_meta.resolution = 0.01;
      map_meta.width = img_msg->width;
      map_meta.height = img_msg->height;

      geometry_msgs::msg::Pose map_pose;
      geometry_msgs::msg::Point origin_point;
      geometry_msgs::msg::Quaternion origin_quat;
      origin_point.x = 0.0;
      origin_point.y = 0.0;
      origin_point.z = 0.0;

      origin_quat.x = 1.0;
      origin_quat.y = 0.0;
      origin_quat.z = 0.0;
      origin_quat.w = 0.0;

      map_pose.position = origin_point;
      map_pose.orientation = origin_quat;

      map_meta.origin = map_pose;

      grid_msg.header = header;
      grid_msg.info = map_meta;
      grid_msg.data = occupancy_grid_data;

      RCLCPP_INFO(this->get_logger(), "Publishing map.");
      publisher_->publish(grid_msg);
    }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);

  auto map_server = std::make_shared<MapServer>();
  rclcpp::spin(map_server);
  rclcpp::shutdown();
  return 0;
}