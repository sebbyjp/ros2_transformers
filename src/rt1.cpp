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
  this->declare_parameter("instruction", "pick block");
  instruction_ = this->get_parameter("instruction").as_string();
  auto world_frame = this->get_parameter("world_frame").as_string();

  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(world_frame, "/rviz_visual_markers", this);

  // Initialize python modules
  // TODO(speralta): Do not do this with the pybind11 embedded interpreter.
    // from_pretrained_ = py::module::import("robo_transformers").attr("inference");
    inference_ = py::module::import("robo_transformers.inference_server").attr("InferenceServer")();
    mp_gil_release_ = std::make_unique<py::gil_scoped_release>();
}

VLA::Feedback::SharedPtr RT1::actionFromObs(std::shared_ptr<RT1Observer> observer)
{

    auto [id, msg] = observer->observe();
      RCLCPP_WARN(this->get_logger(),"img len %d", msg->data.size());
  // convert msg from Image to numpy array
   std::vector<std::vector<std::array<uint8_t, 3>>> image;
   for (int i = 0; i < msg->width; i++)
   {
    image.push_back(std::vector<std::array<uint8_t, 3>>(msg->height));
      for (int j = 0; j < msg->height; j++)
      {
        image[i][j] = std::array<uint8_t, 3>();
        for (int k = 0; k < 3; k++)
        {
          // RCLCPP_WARN(this->get_logger(), "i: %d, j: %d, k: %d", i, j, k);
          image[i][j][k] = msg->data[i * msg->height * 3 + j * 3 + k];
        }
      }
   }
       // TODO(speralta): Read in onnx module and run inference
  auto feedback = std::make_shared<VLA::Feedback>();
      py::gil_scoped_acquire acquire;
      // py::tuple model_tup = from_pretrained_();
      py::dict result = inference_(instruction_, py::cast(image), py::arg("save") = this->get_parameter("visualize").as_bool());
      auto world_vector = result["world_vector"].attr("numpy")().attr("squeeze")().cast<Eigen::Vector3f>();
      py::print(world_vector);
      feedback->gripper_closedness_action = result["gripper_closedness_action"].cast<float>();
      feedback->world_vector[0] = world_vector[0];
      feedback->world_vector[1] = world_vector[1];
      feedback->world_vector[2] = world_vector[2];

      // feedback->world_vector[0] = world_vector.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[0];
      // feedback->world_vector[1] = world_vector.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[1];
      // feedback->world_vector[2] = world_vector.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[2];

      // feedback->world_vector[0] = world_vector[0].cast<float>();
      // feedback->world_vector[1] = world_vector[1].cast<float>();
      // feedback->world_vector[2] = world_vector[2].cast<float>();
     auto rotation_delta = result["rotation_delta"].attr("numpy")().attr("squeeze")().cast<Eigen::Vector3f>();
      py::print(rotation_delta);
      feedback->rotation_delta[0] = rotation_delta[0];
      feedback->rotation_delta[1] = rotation_delta[1];
      feedback->rotation_delta[2] = rotation_delta[2];
      return feedback;

      // feedback->rotation_delta[0] = rotation_delta.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[0];
      // feedback->rotation_delta[1] = rotation_delta.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[1];
      // feedback->rotation_delta[2] = rotation_delta.attr("numpy")().attr("tolist")().cast<std::vector<float>>()[2];
            
      // feedback->rotation_delta[0] = rotation_delta[0].cast<double>();
      // feedback->rotation_delta[1] = rotation_delta[1].cast<double>();
      // feedback->rotation_delta[2] = rotation_delta[2].cast<double>();

      // grasp_list = result[0].cast<std::vector<Eigen::Matrix4d>>();
      // confidence_list = result[1].cast<std::vector<double>>();
    // }
  //   // Convert to python-accessible representation.
  //   PointCloudRGB cloud;
  //   pcl::fromROSMsg(*msg, cloud);
  //   std::vector<std::array<float, 3>> cloud_points;
  //   for (const auto& point : cloud.points)
  //   {
  //     cloud_points.push_back({ point.x, point.y, point.z });
  //   }
  //   RCLCPP_INFO_STREAM(this->get_logger(), "cloud size: " << cloud.points.size());



  //   // Detect grasp poses.
  //   std::vector<geometry_msgs::msg::PoseStamped> grasps;
  //   std::vector<unsigned int> grasp_ids;
  //   RCLCPP_INFO_STREAM(this->get_logger(), "grasp_list size: " << grasp_list.size());
  //   for (unsigned int i = 0; i < grasp_list.size(); i++)
  //   {
  //     grasp_ids.push_back(i);
  //     geometry_msgs::msg::Pose pose;
  //     const auto cgn_grasp = Eigen::Affine3d(grasp_list[i]);
  //     if (visualize)
  //     {
  //       visual_tools_->publishAxisLabeled(tf2::toMsg(cgn_grasp), "cgn_frame");
  //     }

  //     geometry_msgs::msg::PoseStamped grasp;
  //     grasp.header.frame_id = world_frame;
  //     grasp.pose = tf2::toMsg(cgn_grasp * grasp_model_tf.linear());

  //     if (remove_centroid)
  //     {
  //       grasp.pose.position.x += centroid_[0];
  //       grasp.pose.position.y += centroid_[1];
  //       grasp.pose.position.z += centroid_[2];
  //     }
  //     grasps.push_back(grasp);
  //     if (visualize)
  //     {
  //       visual_tools_->publishAxisLabeled(grasp.pose, std::to_string(confidence_list[i]));
  //       visual_tools_->trigger();
  //     }
  //   }
  

  // auto factor = std::pow(-1, count_) * 1.0;
  //   feedback->world_vector[0] = factor * 0.1;
  //   feedback->world_vector[1] = factor * 0.1;
  //   feedback->world_vector[2] = factor * 0.1;

  //   feedback->rotation_delta[0] = 0;
  //   feedback->rotation_delta[1] = 0;
  //   feedback->rotation_delta[2] = 0;
  //   feedback->gripper_closedness_action = factor * 0.5;
  //     count_++;
  
  //   for (auto id : grasp_ids)
  //   {
  //     feedback->grasp_candidates.emplace_back(grasps[id]);
  //     feedback->costs.emplace_back(1.0 / confidence_list[id]);
  //   }

  // return feedback;
}

std::unique_ptr<Image> RT1::obsFromSrcs(std::shared_ptr<Image> msg)
{
  auto image = std::make_unique<Image>();
  *image = *msg;
  return image;
}
}  // namespace r2t

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  r2t::RT1 server(options);
  server.run();
  rclcpp::shutdown();
  return 0;
}
