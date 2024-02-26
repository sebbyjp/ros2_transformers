// Copyright 2024 Sebastian Peralta
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


#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <ros2_agents/agent.hpp>
#include <ros2_agents/observer.hpp>
#include <ros2_agents/actor.hpp>
#include <ros2_agents_interfaces/action/mbodied.hpp>
namespace mbodied
{

typedef Observer<std::vector<sensor_msgs::msg::Image>, sensor_msgs::msg::Image, sensor_msgs::msg::Image> OctoObserver;
typedef Agent<std::vector<sensor_msgs::msg::Image>, ros2_agents_interfaces::action::Mbodied,
    sensor_msgs::msg::Image,  sensor_msgs::msg::Image> OctoAgent;

class Octo : public OctoAgent
{
public:
  explicit Octo(rclcpp::NodeOptions & options);

  ros2_agents_interfaces::action::Mbodied::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<OctoObserver> observer) override;

    std::unique_ptr<std::vector<sensor_msgs::msg::Image>> 
  obsFromSrcs(std::shared_ptr<sensor_msgs::msg::Image> image_primary, std::shared_ptr<sensor_msgs::msg::Image> image_wrist)
  override;

  ~Octo()
  {
    inference_.release();
  }

private:

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // TODO(speralta): Do not use global pybind vairables.
  pybind11::function inference_;

  pybind11::scoped_interpreter py_guard_;
  std::unique_ptr<pybind11::gil_scoped_release> mp_gil_release_;
  int count_ = 0;

};
}  // namespace mbodied
