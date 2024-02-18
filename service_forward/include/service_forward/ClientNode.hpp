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

#ifndef SERVICE_FORWARD__CLIENTNODE_HPP_
#define SERVICE_FORWARD__CLIENTNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "service_forward_interfaces/srv/GetInformation.hpp"

namespace service_forward
{

class ClientNode : public rclcpp::Node
{
public:
  ClientNode();
  std::optional<service_forward_interfaces::srv::GetInformation::Response> call_client(double distance);
private:
  rclcpp::Client<service_forward_interfaces::srv::GetInformation>::SharedPtr client_;
};

}  //  namespace service_forward

#endif  // SERVICE_FORWARD__CLIENTNODE_HPP_