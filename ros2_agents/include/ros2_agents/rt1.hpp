// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

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

typedef Observer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> RT1Observer;
typedef Agent<sensor_msgs::msg::Image, ros2_agents_interfaces::action::Mbodied,
    sensor_msgs::msg::Image>
  RT1Agent;
class RT1 : public RT1Agent
{
public:
  explicit RT1(rclcpp::NodeOptions & options);

  ros2_agents_interfaces::action::Mbodied::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<RT1Observer> observer) override;

  sensor_msgs::msg::Image::UniquePtr
  obsFromSrcs(std::shared_ptr<sensor_msgs::msg::Image> msg)
  override;

  ~RT1()
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
  std::string instruction_;
};
}  // namespace mbodied
