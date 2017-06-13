// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "intra_process_demo/image_pipeline/camera_node.hpp"
#include "intra_process_demo/image_pipeline/image_view_node.hpp"
#include "astra_camera/astra_driver.h"
#include "linemod_basic_detector/linemod_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  if(argc < 3) {
    std::cerr << "usage is: linemod_pipeline templates_file <r|d|b>"
              << std::endl
              << "where:" << std::endl
              << "\td: use depth modality only" << std::endl
              << "\tr: use rgb modality only" << std::endl
              << "\tb: use both modalities" << std::endl;
    return 1;
  }
  bool depth = (argv[2][0] == 'd' || argv[2][0] == 'b');
  bool rgb = (argv[2][0] == 'r' || argv[2][0] == 'b');
  if(!depth && !rgb) {
    std::cerr << "you must specify usage of at least one of depth or rgb." <<
    std::endl;
    return 1;
  }
  // Connect the nodes as a pipeline:
  // astra_node -> linemod_node -> image_view_node

  // RGB
  size_t rgb_width = 640;
  size_t rgb_height = 480;
  double framerate = 30;

  // Depth
  size_t depth_width = 640;
  size_t depth_height = 480;
  double depth_framerate = 30;
  astra_wrapper::PixelFormat depth_format =
      astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM;

  rclcpp::node::Node::SharedPtr astra_node =
      rclcpp::node::Node::make_shared("astra_camera");
  rclcpp::node::Node::SharedPtr astra_private_node =
      rclcpp::node::Node::make_shared("astra_camera_");
  astra_private_node->set_parameter_if_not_set("depth_registration", true);

  astra_wrapper::AstraDriver drv(astra_node, astra_private_node,
    rgb_width, rgb_height, framerate,
    depth_width, depth_height, depth_framerate, depth_format);

  std::cout << "starting linemod node with template file " << argv[1] << std::endl;
  auto linemod_node =
    std::make_shared<LinemodNode>(argv[1], rgb, depth,
                                  "image", "depth",
                                  "image_with_detections",
                                  "detections",
                                  "linemod_node");
  auto image_view_node =
    std::make_shared<ImageViewNode>("image_with_detections", "image_view_node",
                                    false);

  // executor.add_node(camera_node);
  executor.add_node(astra_node);
  executor.add_node(astra_private_node);
  executor.add_node(linemod_node);
  executor.add_node(image_view_node);

  executor.spin();
  return 0;
}
