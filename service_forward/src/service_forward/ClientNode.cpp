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

#include "service_forward/ClientNode.hpp"

using namespace std::chrono_literals;

namespace service_forward
{

ClientNode::ClientNode()
: Node("my_client_node")
{
  client_ = this->create_client<service_forward_interfaces::srv::GetInformation>("my_service");
}

std::optional<service_forward_interfaces::srv::GetInformation::Response>
ClientNode::call_client(double distance)
{
  while (!client_->wait_for_service(1s)) {
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<service_forward_interfaces::srv::GetInformation::Request>();
  request->distance = distance;

  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "service call failed :(");
    client_->remove_pending_request(result);
    return {};
  }

  return *result.get();
}
}  //  namespace service_forward