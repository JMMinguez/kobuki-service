// Copyright 2024 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"

#include "service_forward/ServerNode.hpp"
#include "service_forward_interfaces/srv/GetInformation.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace service_forward
{

ServerNode::ServerNode()
: Node("my_server_node")
{
  server_ = create_service<service_forward_interfaces::srv::GetInformation>(
    "my_service",
    std::bind(&ServerNode::move_callback, this, _1, _2));

  vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void
ServerNode::move_callback(
  const service_forward_interfaces::srv::GetInformation::Request::SharedPtr request,
  service_forward_interfaces::srv::GetInformation::Response::SharedPtr response)
{
  // If new message recieve before the previous distance is reached, the robot stops to calculate the tf correctly
  l_vel_.linear.x = STOP_SPEED;
  vel_->publish(l_vel_);

  RCLCPP_INFO(get_logger(), "Received advance command: '%f'", request->distance);

  std::string error;
  tf2::Stamped<tf2::Transform> odom2bf;
  if (start_){
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf);
    }
    actual_distance_ = 0.0;
    start_ = !start_;
  }

  tf2::Transform odom2bf_inverse = odom2bf.inverse();

  tf2::Stamped<tf2::Transform> odom2bfa;
  while (actual_distance_ < distance){
    start_ = start_;

    // Gets the tf from start 'odom' and actual 'base_footprint'
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bfa_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bfa_msg, odom2bfa);

    // Gets the tf from start 'base_footprint' and actual 'base_footprint'
    tf2::Transform bf2bfa = odom2bf_inverse * odom2bfa;
    
    //  Extracts the x and y coordinates from the obtained transformation.
    double x = bf2bfa.getOrigin().x();
    double y = bf2bfa.getOrigin().y();

    //  Calculate the distance between (0,0) and (x,y)
    distance_ = sqrt(x * x + y * y);
    }

    // Move the robot
    l_vel_.linear.x = MOVE_SPEED;
    vel_->publish(l_vel_);
  }

  // When the distance is reached, the robot stops
  l_vel_.linear.x = STOP_SPEED;
  vel_->publish(l_vel_);
  
  response->response = std_msgs::msg::Empty();
}
}