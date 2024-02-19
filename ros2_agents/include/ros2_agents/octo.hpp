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

#include <ros2_agents_interfaces/action/vla.hpp>
namespace mbodied
{

typedef mbodied::Observer<std::vector<sensor_msgs::msg::Image>, sensor_msgs::msg::Image, sensor_msgs::msg::Image> OctoObserver;
typedef mbodied::Agent<std::vector<sensor_msgs::msg::Image>, ros2_agents_interfaces::action::VLA,
    sensor_msgs::msg::Image,  sensor_msgs::msg::Image>
  OctoAgent;
class Octo : public OctoAgent
{
public:
  explicit Octo(rclcpp::NodeOptions & options);

  ros2_agents_interfaces::action::VLA::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<OctoObserver> observer) override;

    std::unique_ptr<std::vector<sensor_msgs::msg::Image>> 
  obsFromSrcs(std::shared_ptr<sensor_msgs::msg::Image> image_primary, std::shared_ptr<sensor_msgs::msg::Image> image_wrist)
  override;

  ~Octo()
  {
    inference_.release();
  }

private:

  std::string get_instruction() {
    return this->get_parameter("instruction").as_string();
  }

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // TODO(speralta): Do not use global pybind vairables.
  pybind11::function inference_;

  pybind11::scoped_interpreter py_guard_;
  std::unique_ptr<pybind11::gil_scoped_release> mp_gil_release_;
  int count_ = 0;

};
}  // namespace mbodied
