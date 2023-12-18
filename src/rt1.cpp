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
#include <dgl_ros_moveit/stages/action_base.hpp>

using dgl_ros_interfaces::action::VLA;
using sensor_msgs::msg::Image;
// typedef  moveit::task_constructor::stages::ActionBase<VLA> ActBase;

// class ActClient : public ActBase {
//   VLAtypedef typename VLA::Feedback Feedback;
//   typedef typename VLA::Result Result;
//   typedef typename rclcpp_action::ClientGoalHandle<VLA>::SharedPtr GoalHandleSharedPtr;
//   typedef typename rclcpp_action::ClientGoalHandle<VLA>::WrappedResult WrappedResult;
//    ActClienet(): ActBase("vla", false) {
//    }



//   /* @brief Called when goal becomes active */
//   void goal_response_callback(const GoalHandleSharedPtr& goal_handle) {

//   }

//   /**
//    * @brief Called every time feedback is received for the goal
//    * @param feedback - pointer to the feedback message
//    */
//   void feedback_callback(const GoalHandleSharedPtr goal_handle,
//                                  const std::shared_ptr<const Feedback> feedback) {



//                                  }

//   /**
//    * @brief Called once when the goal completes
//    * @param state - state info for goal
//    * @param result - pointer to result message
//    */

//   void result_callback(const WrappedResult& result) {}

// }





#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rt1");



namespace py = pybind11;
namespace r2t
{

  
RT1::RT1(rclcpp::NodeOptions& options) : RT1Agent(options), py_guard_()
{
  this->declare_parameter("visualize", false);
  auto world_frame = this->get_parameter("world_frame").as_string();


  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(world_frame, "/rviz_visual_markers", this);

  // Initialize python interpreter.
  // TODO(speralta): Do not do this with the pybind11 embedded interpreter.
//   from_pretrained_ = py::module::import("cgn_pytorch").attr("from_pretrained");
//   inference_ = py::module::import("cgn_pytorch").attr("inference");
//   mp_gil_release_ = std::make_unique<py::gil_scoped_release>();
}

VLA::Feedback::SharedPtr RT1::actionFromObs(std::shared_ptr<RT1Observer> observer)
{
//   auto success_threshold = this->get_parameter("success_threshold").as_double();
//   auto visualize = this->get_parameter("visualize").as_bool();
//   auto max_grasps = this->get_parameter("max_grasps").as_int();
//   auto gripper_depth = this->get_parameter("gripper_depth").as_double();
//   auto gripper_width = this->get_parameter("gripper_width").as_double();

//   auto remove_centroid = this->get_parameter("remove_centroid").as_bool();
//   auto world_frame = this->get_parameter("world_frame").as_string();
//   auto grasp_model_tf = dgl::util::isometryFromXYZRPY(this->get_parameter("grasp_model_tf").as_double_array());

//   auto [id, msg] = observer->observe();

//   // Convert to python-accessible representation.
//   PointCloudRGB cloud;
//   pcl::fromROSMsg(*msg, cloud);
//   std::vector<std::array<float, 3>> cloud_points;
//   for (const auto& point : cloud.points)
//   {
//     cloud_points.push_back({ point.x, point.y, point.z });
//   }
//   RCLCPP_INFO_STREAM(this->get_logger(), "cloud size: " << cloud.points.size());

//   // TODO(speralta): Read in onnx module and run inference
//   // For now, access cgn-pytorch in an embedded python interpreter.
//   std::vector<Eigen::Matrix4d> grasp_list;
//   std::vector<double> confidence_list;
//   {
//     py::gil_scoped_acquire acquire;
//     py::tuple model_tup = from_pretrained_();
//     py::tuple result = inference_(model_tup[0], py::cast(cloud_points), success_threshold, visualize, max_grasps,
//                                   gripper_depth, gripper_width);
//     grasp_list = result[0].cast<std::vector<Eigen::Matrix4d>>();
//     confidence_list = result[1].cast<std::vector<double>>();
//   }

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

  auto feedback = std::make_shared<VLA::Feedback>();
//   for (auto id : grasp_ids)
//   {
//     feedback->grasp_candidates.emplace_back(grasps[id]);
//     feedback->costs.emplace_back(1.0 / confidence_list[id]);
//   }

  return feedback;
}

std::unique_ptr<Image> RT1::obsFromSrcs(std::shared_ptr<Image> msg)
{
  auto image = std::make_unique<Image>();
  *image = *msg;
  return image;
}

  void RT1::run()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(actor_);
    exec.add_node(observer_);
    std::thread([&exec]() { exec.spin(); }).detach();
  }
}  // namespace r2t

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  r2t::RT1 server(options);
  server.run();


  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("motion_planning_pipeline_tutorial", node_options);
  auto client = rclcpp_action::create_client<ActionT>(node, "vla");


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
  // a RobotModel and a PlanningScene.
  //
  // We will start by instantiating a
  // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
  // object, which will look up the robot description on the ROS
  // parameter server and construct a
  // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
  // for us to use.
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  // Using the RobotModelLoader, we can construct a planning scene monitor that
  // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
  // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
  // startStateMonitor to fully initialize the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));

  /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                       the internal planning scene accordingly */
  psm->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                                world geometry, collision objects and optionally octomaps */
  psm->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision objects
                        and update the internal planning scene accordingly*/
  psm->startStateMonitor();

  /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
     for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
     RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
     group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");

  // We can now setup the PlanningPipeline object, which will use the ROS parameter server
  // to determine the set of request adapters and the planning plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "move_group_tutorial", psm);
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

  // /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  // visual_tools.trigger();

  // /* We can also use visual_tools to wait for user input */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  
  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  req.pipeline_id = "ompl";
  req.planner_id = "RRTConnectkConfigDefault";
  req.allowed_planning_time = 1.0;
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.1);
  std::vector<double> tolerance_angle(3, 0.1);

  // We will create the request as a constraint using a helper
  // function available from the
  // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
  // package.
  req.group_name = "panda_arm";
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    /* Check that the planning was successful */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
      node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, setup a joint space goal
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::msg::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  /* Visualize the trajectory */
  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see two planned trajectories in series
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Using a Planning Request Adapter
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // A planning request adapter allows us to specify a series of operations that
  // should happen either before planning takes place or after the planning
  // has been done on the resultant path

  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, set one of the joints slightly outside its upper limit
  const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
  const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
  robot_state->setJointPositions(joint_model, tmp_values);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  /* Visualize the trajectory */
  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  /* Now you should see three planned trajectories in series*/
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  
  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

  RCLCPP_INFO(LOGGER, "Done");

  rclcpp::shutdown();
  return 0;
}
  