// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos = qos.best_effort();
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb", qos);

    {
      int width = 1280;
      int height = 720;
      frame_.resize(width * height);
      RCLCPP_INFO_STREAM(this->get_logger(), "Testing Frame Size: " << width << " * " << height << " = " << frame_.size() << " bytes.");
    }
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ImagePublisher::publish_image, this));
    inspection_timer_ = this->create_wall_timer(
      1000ms, std::bind(&ImagePublisher::inspect_metrics, this));
  }

private:
  void publish_image()
  {
    auto msg = sensor_msgs::msg::CompressedImage();
    msg.format = "jpg";
    msg.data = frame_;
    publisher_->publish(msg);
    count_++;
  }

  void inspect_metrics()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "FPS: " << count_);
    count_ = 0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr inspection_timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  std::vector<uint8_t> frame_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
