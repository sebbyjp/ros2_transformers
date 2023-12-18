// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <dgl_ros/agent.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/actor.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <dgl_ros_interfaces/action/vla.hpp>
namespace r2t
{

typedef dgl::Observer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> RT1Observer;
typedef dgl::Agent<sensor_msgs::msg::Image, dgl_ros_interfaces::action::VLA,
                   sensor_msgs::msg::Image>
    RT1Agent;
class RT1 : public RT1Agent
{
public:
  RT1(rclcpp::NodeOptions& options);

  dgl_ros_interfaces::action::VLA::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<RT1Observer> observer) override;

  sensor_msgs::msg::Image::UniquePtr obsFromSrcs(std::shared_ptr<sensor_msgs::msg::Image> msg) override;

  ~RT1()
  {
    inference_.release();
    from_pretrained_.release();
  }

  void run() override;


private:

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // TODO(speralta): Do not use global pybind vairables.
  pybind11::function inference_;
  pybind11::function from_pretrained_;

  pybind11::scoped_interpreter py_guard_;
  std::unique_ptr<pybind11::gil_scoped_release> mp_gil_release_;
};
}  // namespace r2t