// Copyright 2023 Yunlong Feng
// Copyright 2025 Lihan Chen
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

#ifndef ARMOR_DETECTOR_OPENVINO__DETECTOR_OPENVINO_NODE_HPP_
#define ARMOR_DETECTOR_OPENVINO__DETECTOR_OPENVINO_NODE_HPP_

#include <future>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "armor_detector_openvino/detector_openvino.hpp"
#include "armor_detector_openvino/mono_measure_tool.hpp"
#include "armor_detector_openvino/types.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rm_auto_aim
{

class ArmorDetectorOpenvinoNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorOpenvinoNode(const rclcpp::NodeOptions & options);

private:
  void initDetector();

  void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  void openvinoDetectCallback(
    const std::vector<ArmorObject> & objs, int64_t timestamp_nanosec, const cv::Mat & src_img);

  void publishMarkers(bool is_empty);

  // Debug functions
  void createDebugPublishers();

  void destroyDebugPublishers();

private:
  std::string camera_name_;
  std::string transport_type_;
  std::string frame_id_;
  // OpenVINO Detector
  int detect_color_;  // 0: red, 1: blue
  std::unique_ptr<DetectorOpenVino> detector_;
  std::queue<std::future<bool>> detect_requests_;
  // Camera info
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<MonoMeasureTool> measure_tool_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;

  // ROS
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::shared_ptr<image_transport::Subscriber> img_sub_;

  // Debug publishers
  bool debug_mode_{false};
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher debug_img_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR_OPENVINO__DETECTOR_OPENVINO_NODE_HPP_
