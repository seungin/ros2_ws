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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos = qos.best_effort();
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("rgb", qos,
      std::bind(&ImageSubscriber::subscribe_image, this, _1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&ImageSubscriber::inspect_metrics, this));
  }

private:
  void subscribe_image(sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    (void)msg;
    count_++;
    RCLCPP_INFO_STREAM(this->get_logger(), "-- received images: " << count_);
  }
  void inspect_metrics()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "FPS: " << count_);
    count_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
