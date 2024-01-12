#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dgl_ros_interfaces/action/vla.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Geometry>
#include <ros2_transformers/rt1.hpp>
using dgl_ros_interfaces::action::VLA;
using sensor_msgs::msg::Image;

namespace py = pybind11;
namespace r2t
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
      "InferenceServer")(py::arg("model_type") = model_type, py::arg("weights_key") = weights_key, py::arg("dummy") = dummy);
    mp_gil_release_                              = std::make_unique<py::gil_scoped_release>();
  }

  VLA::Feedback::SharedPtr RT1::actionFromObs(std::shared_ptr<RT1Observer>observer)
  {
    auto [id, msg] = observer->observe();

    // convert msg from Image to numpy array
    std::vector<std::vector<std::array<uint8_t, 3> > > image;

    for (int i = 0; i < msg->height; i++)
    {
      image.push_back(std::vector<std::array<uint8_t, 3> >(msg->width));

      for (int j = 0; j < msg->width; j++)
      {
        image[i][j] = std::array<uint8_t, 3>();

        for (int k = 0; k < 3; k++)
        {
          image[i][j][k] = msg->data[i * msg->width * 3 + j * 3 + k];
        }
      }
    }

    bool visualize = this->get_parameter("visualize").as_bool();
    auto feedback  = std::make_shared<VLA::Feedback>();

    // TODO(speralta): Read in onnx module and run inference
    py::gil_scoped_acquire acquire;
    py::dict result = inference_(visualize,
                                 py::arg("instruction") =instruction_,
                                    py::arg("image") =           py::cast(image));

    auto world_vector =
      result["world_vector"].attr("squeeze")().cast<Eigen::Vector3f>();

    py::print(world_vector);
    feedback->gripper_closedness_action = result["gripper_closedness_action"].cast<float>();
    feedback->world_vector[0]           = world_vector[0];
    feedback->world_vector[1]           = world_vector[1];
    feedback->world_vector[2]           = world_vector[2];

    auto rotation_delta =
      result["rotation_delta"].attr("squeeze")().cast<Eigen::Vector3f>();

    py::print(rotation_delta);
    feedback->rotation_delta[0] = rotation_delta[0];
    feedback->rotation_delta[1] = rotation_delta[1];
    feedback->rotation_delta[2] = rotation_delta[2];
    RCLCPP_ERROR(this->get_logger(), "feedback: %f, %f, %f, %f, %f, %f, %f",
                 feedback->gripper_closedness_action,
                 feedback->world_vector[0],
                 feedback->world_vector[1],
                 feedback->world_vector[2],
                 feedback->rotation_delta[0],
                 feedback->rotation_delta[1],
                 feedback->rotation_delta[2]);
    return feedback;
  }

  std::unique_ptr<Image>RT1::obsFromSrcs(std::shared_ptr<Image>msg)
  {
    auto image = std::make_unique<Image>();

    *image = *msg;
    return image;
  }
} // namespace r2t

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  r2t::RT1 server(options);

  server.run();
  rclcpp::shutdown();
  return 0;
}
