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
#include <dgl_ros/agent.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/actor.hpp>

#include <dgl_ros_interfaces/action/vla.hpp>
namespace r2t
{

typedef dgl::Observer<std::vector<sensor_msgs::msg::Image>, sensor_msgs::msg::Image, sensor_msgs::msg::Image> OctoObserver;
typedef dgl::Agent<std::vector<sensor_msgs::msg::Image>, dgl_ros_interfaces::action::VLA,
    sensor_msgs::msg::Image,  sensor_msgs::msg::Image>
  OctoAgent;
class Octo : public OctoAgent
{
public:
  explicit Octo(rclcpp::NodeOptions & options);

  dgl_ros_interfaces::action::VLA::Feedback::SharedPtr
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
}  // namespace r2t
