// Copyright 2023 Yunlong Feng
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
#include <memory>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "armor_detector_openvino/detector_openvino_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

TEST(ArmorDetecterNode, NodeStartupTest)
{
  rclcpp::init(0, nullptr);

  auto node_options = rclcpp::NodeOptions();
  std::string model_path = ament_index_cpp::get_package_share_directory("armor_detector_openvino") +
                           "/model/opt-1208-001.onnx";
  node_options.append_parameter_override("detector.model_path", model_path);
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorOpenvinoNode>(node_options);

  auto spin_thread = std::thread([&]() { rclcpp::spin(node); });
  std::this_thread::sleep_for(1000ms);
  rclcpp::shutdown();
  spin_thread.join();

  SUCCEED();
}
