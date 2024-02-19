#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros2_agents_interfaces/action/Mbodied.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Geometry>
#include <ros2_agents/octo.hpp>
using ros2_agents_interfaces::action::Mbodied;
using sensor_msgs::msg::Image;

std::vector<std::vector<std::array<uint8_t, 3> > >image_to_arr(Image *msg)
{
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
  return image;
}

namespace py = pybind11;
namespace mbodied
{
  Octo::Octo(rclcpp::NodeOptions& options) : OctoAgent(options), py_guard_()
  {
    this->declare_parameter("visualize", true);
    this->declare_parameter("instruction", "pick block");

    // Whether or not to load the inference server or just pass through.
    this->declare_parameter("dummy", false);
    this->declare_parameter("weights_key", "octo-base");
    this->declare_parameter("model_type", "octo");
    this->declare_parameter("py_log_level", "INFO");

    // Initialize parameters
    bool dummy = this->get_parameter("dummy").as_bool();

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
    py::module::import("absl").attr("logging").attr("set_verbosity")(this->get_parameter(
                                                                       "py_log_level").as_string());

    // Initialize GIL)
    mp_gil_release_ = std::make_unique<py::gil_scoped_release>();
  }

  Mbodied::Feedback::SharedPtr Octo::actionFromObs(std::shared_ptr<OctoObserver>observer)
  {
    auto [id, images] = observer->observe();
    auto image_primary = image_to_arr(&(*images)[0]);
    auto image_wrist   =  image_to_arr(&(*images)[1]);


    bool visualize = this->get_parameter("visualize").as_bool();
    auto feedback  = std::make_shared<Mbodied::Feedback>();

    // TODO(speralta): Read in onnx module and run inference
    py::gil_scoped_acquire acquire;
    py::dict result = inference_(visualize,
                                 py::arg("instruction") = get_instruction(),
                                 py::arg("image")       =           py::cast(image_primary),
                                 py::arg("image_wrist") = py::cast(image_wrist));



    feedback->actions[0].left_hand.x = result["x"].cast<float>();
    feedback->actions[0].left_hand.y = result["y"].cast<float>();
    feedback->actions[0].left_hand.z = result["z"].cast<float>();


    feedback->actions[0].left_hand.roll  = result["roll"].cast<float>();
    feedback->actions[0].left_hand.pitch = result["pitch"].cast<float>();
    feedback->actions[0].left_hand.yaw   = result["yaw"].cast<float>();

    feedback->actions[0].left_hand.grasp = result["grasp"].cast<float>();

    RCLCPP_WARN(this->get_logger(),
                "feedback: %f, %f, %f, %f, %f, %f, %f",
                feedback->actions[0].left_hand.x,
                feedback->actions[0].left_hand.y,
                feedback->actions[0].left_hand.z,
                feedback->actions[0].left_hand.roll,
                feedback->actions[0].left_hand.pitch,
                feedback->actions[0].left_hand.yaw,
                feedback->actions[0].left_hand.grasp);
    return feedback;
  }

  std::unique_ptr<std::vector<Image> >Octo::obsFromSrcs(std::shared_ptr<Image>image_primary,
                                                        std::shared_ptr<Image>image_wrist)
  {
    auto obs =  std::make_unique<std::vector<Image> >();

    obs->push_back(*image_primary);
    obs->push_back(*image_wrist);
    return obs;
  }
} // namespace mbodied

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  mbodied::Octo server(options);

  server.run();
  rclcpp::shutdown();
  return 0;
}
