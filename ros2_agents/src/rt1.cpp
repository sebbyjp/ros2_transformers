#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros2_agents_interfaces/action/mbodied.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Geometry>
#include <ros2_agents/rt1.hpp>
using ros2_agents_interfaces::action::Mbodied;
using sensor_msgs::msg::Image;

namespace py = pybind11;
namespace mbodied
{
  RT1::RT1(rclcpp::NodeOptions& options) : RT1Agent(options), py_guard_()
  {
    this->declare_parameter("visualize", true);
    this->declare_parameter("default_instruction", "pick block");

    // Whether or not to load the inference server or just pass through.
    this->declare_parameter("dummy", false);
    this->declare_parameter("weights_key", "rt1multirobot");
    this->declare_parameter("model_type", "rt1");

    // Initialize parameters
    bool dummy = this->get_parameter("dummy").as_bool();

    instruction_ = this->get_parameter("default_instruction").as_string();
    auto world_frame = this->get_parameter("world_frame").as_string();
    auto weights_key = this->get_parameter("weights_key").as_string();
    auto model_type  = this->get_parameter("model_type").as_string();

    // Initialize rviz_visual_tools

    visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(world_frame,
                                                                         "/rviz_visual_markers",
                                                                         this);

    // Initialize python modules
    // TODO(speralta): Do not do this with the pybind11 embedded interpreter.
    inference_ = py::module::import(
      "robo_transformers.inference_server").attr(
      "InferenceServer")(py::arg("model_type") = model_type, py::arg("weights_key") = weights_key,
                         py::arg("dummy")      = dummy);
    mp_gil_release_                            = std::make_unique<py::gil_scoped_release>();
  }

  Mbodied::Feedback::SharedPtr RT1::actionFromObs(std::shared_ptr<RT1Observer>observer)
  {
    auto [id, msg] = observer->observe();

    // convert msg from Image to numpy array
    std::vector<std::vector<std::array<uint8_t, 3> > > image;

    for (int i = 0; i < static_cast<int>(msg->height); i++)
    {
      image.push_back(std::vector<std::array<uint8_t, 3> >(msg->width));

      for (int j = 0; j < static_cast<int>(msg->width); j++)
      {
        image[i][j] = std::array<uint8_t, 3>();

        for (int k = 0; k < 3; k++)
        {
          image[i][j][k] = msg->data[i * msg->width * 3 + j * 3 + k];
        }
      }
    }

    bool visualize = this->get_parameter("visualize").as_bool();
    auto feedback  = std::make_shared<Mbodied::Feedback>();

    // TODO(speralta): Read in onnx module and run inference
    py::gil_scoped_acquire acquire;
    py::dict result = inference_(visualize,
                                 py::arg("instruction") = get_instruction(),
                                 py::arg("image")       =           py::cast(image));

    auto world_vector =
      result["world_vector"].attr("squeeze")().cast<Eigen::Vector3f>();

    py::print(world_vector);
    feedback->actions[0].left_hand.grasp = result["gripper_closedness_action"].cast<float>();
    feedback->actions[0].left_hand.x     = world_vector[0];
    feedback->actions[0].left_hand.y     = world_vector[1];
    feedback->actions[0].left_hand.z     = world_vector[2];

    auto rotation_delta =
      result["rotation_delta"].attr("squeeze")().cast<Eigen::Vector3f>();

    py::print(rotation_delta);
    feedback->actions[0].left_hand.roll = rotation_delta[0];
    feedback->actions[0].left_hand.pitch = rotation_delta[1];
    feedback->actions[0].left_hand.yaw   = rotation_delta[2];
    RCLCPP_INFO(this->get_logger(), "feedback: %f, %f, %f, %f, %f, %f, %f",
                feedback->actions[0].left_hand.x,
                feedback->actions[0].left_hand.y,
                feedback->actions[0].left_hand.z,
                feedback->actions[0].left_hand.roll,
                feedback->actions[0].left_hand.pitch,
                feedback->actions[0].left_hand.yaw,
                feedback->actions[0].left_hand.grasp);
    return feedback;
  }

  std::unique_ptr<Image>RT1::obsFromSrcs(std::shared_ptr<Image>msg)
  {
    auto image = std::make_unique<Image>();

    *image = *msg;
    return image;
  }
} // namespace mbodied

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  mbodied::RT1 server(options);

  server.run();
  rclcpp::shutdown();
  return 0;
}
