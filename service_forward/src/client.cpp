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

#include <memory>

#include "service_forward/ClientNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usa: ros2 run service_forward client_node_main distance" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<service_forward::ClientNode>();

  double distance = std::stod(argv[1]);
  auto result = node->call_client(distance);
  if (result.has_value()) {
    std::cout << "Service call succeeded" << std::endl;
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}