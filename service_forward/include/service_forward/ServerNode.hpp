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

#ifndef SERVICE_FORWARD__SERVERNODE_HPP_
#define SERVICE_FORWARD__SERVERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "service_forward_interfaces/srv/get_information.hpp"

namespace service_forward
{

class ServerNode : public rclcpp::Node
{
public:
  ServerNode();
  void move_callback(
    const service_forward_interfaces::srv::GetInformation::Request::SharedPtr request,
    service_forward_interfaces::srv::GetInformation::Response::SharedPtr response);
  double distance;
  bool is_moving;

private:
  void transform_callback();
  void move_forward();
  void go_state(int new_state);
  bool check_distance();

  rclcpp::Service<service_forward_interfaces::srv::GetInformation>::SharedPtr server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
  geometry_msgs::msg::Twist l_vel_;

  rclcpp::TimerBase::SharedPtr timer_pos_check_;
  rclcpp::TimerBase::SharedPtr timer_publish_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> odom2bf_;
  tf2::Transform odom2bf_inverse;

  bool start_;
  double actual_distance_;

  const float MOVE_SPEED = 0.3;
  const float STOP_SPEED = 0.0;

  static const int FORWARD = 0;
  static const int STOP = 1;
  int state_;
};

}  //  namespace service_forward

#endif  // SERVICE_FORWARD__SERVERNODE_HPP_
