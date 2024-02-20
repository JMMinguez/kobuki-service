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

#include <chrono>

#include "service_forward/ServerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "service_forward_interfaces/srv/get_information.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace service_forward
{

ServerNode::ServerNode()
: Node("my_server_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  is_moving(true)
{
  server_ = create_service<service_forward_interfaces::srv::GetInformation>(
    "my_service",
    std::bind(&ServerNode::move_callback, this, _1, _2));

  vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_pos_check_ = create_wall_timer(
    50ms, std::bind(&ServerNode::transform_callback, this));
  timer_publish_ = create_wall_timer(
    50ms, std::bind(&ServerNode::move_forward, this));
}

void
ServerNode::move_callback(
  const service_forward_interfaces::srv::GetInformation::Request::SharedPtr request,
  service_forward_interfaces::srv::GetInformation::Response::SharedPtr response)
{

  RCLCPP_INFO(get_logger(), "Received advance command: '%f'", request->distance);
  distance = request->distance;
  std::cerr << "Distance_request: \t" << distance << std::endl;
  start_ = true;

  if (!is_moving) {
    response->response = std_msgs::msg::Empty();
  }
}

void
ServerNode::transform_callback()
{
  tf2::Stamped<tf2::Transform> odom2bfa;
  std::string error;

  if (start_) {
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
      auto odom2bf_msg = tf_buffer_.lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf_);
    }

    odom2bf_inverse = odom2bf_.inverse();
    start_ = false;
  }
  // Gets the tf from start 'odom' and actual 'base_footprint'
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bfa_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bfa_msg, odom2bfa);

    // Gets the tf from start 'base_footprint' and actual 'base_footprint'
    tf2::Transform bf2bfa = odom2bf_inverse * odom2bfa;

    std::cerr << "BF: \t" << odom2bf_.getOrigin().x() << " " << odom2bf_.getOrigin().y() <<
      std::endl;
    std::cerr << "BFa: \t" << odom2bfa.getOrigin().x() << " " << odom2bfa.getOrigin().y() <<
      std::endl;
    std::cerr << "TF: \t" << bf2bfa.getOrigin().x() << " " << bf2bfa.getOrigin().y() << std::endl;

    //  Extracts the x and y coordinates from the obtained transformation.
    double x = bf2bfa.getOrigin().x();
    double y = bf2bfa.getOrigin().y();

    //  Calculate the distance between (0,0) and (x,y)
    actual_distance_ = sqrt(x * x + y * y);
  }
}

void
ServerNode::move_forward()
{
  //  FSM
  switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "Moving forward!");
      l_vel_.linear.x = MOVE_SPEED;
      vel_->publish(l_vel_);
      is_moving = true;

      if (check_distance()) {
        go_state(STOP);
      }
      break;

    case STOP:
      RCLCPP_INFO(get_logger(), "STOP!");
      l_vel_.linear.x = STOP_SPEED;
      vel_->publish(l_vel_);
      is_moving = false;

      if (!check_distance()) {
        go_state(FORWARD);
      }
      break;
  }
}

void
ServerNode::go_state(int new_state)
{
  //  Change state
  state_ = new_state;
}

bool
ServerNode::check_distance()
{
  //  Check distance
  return actual_distance_ >= distance;
}

}  //  namespace service_forward
