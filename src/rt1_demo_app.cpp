#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <dgl_ros_interfaces/action/vla.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
namespace mtc = moveit::task_constructor;
using geometry_msgs::msg::TwistStamped;

geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose)
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose[0];
  p.position.y = pose[1];
  p.position.z = pose[2];
  Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// // Clamp value between lo, hi


// static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

// std::shared_ptr<mtc::solvers::PlannerInterface> get_planner(
//   const std::string & name,
//   rclcpp::Node::SharedPtr node)
// {
//   if (name == "ompl") {
//     return std::make_shared<mtc::solvers::PipelinePlanner>(node, name);
//   } else if (name == "chomp") {
//     return std::make_shared<mtc::solvers::PipelinePlanner>(node, name);
//   } else if (name == "cartesian") {
//     auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//     cartesian_planner->setMaxVelocityScalingFactor(1.0);
//     cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//     cartesian_planner->setStepSize(.02);
//     return cartesian_planner;
//   } else {
//     return std::make_shared<mtc::solvers::JointInterpolationPlanner>();
//   }

// }

// class AgentClient
// {
// public:
//   AgentClient(const rclcpp::NodeOptions & options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

//   void act();

// private:
//   // Executes arbitrary agent.
//   mtc::Task plan();
//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;
//   std::shared_ptr<mtc::solvers::PlannerInterface> hand_planner_;
//   std::shared_ptr<mtc::solvers::PlannerInterface> arm_planner_;

// };

// AgentClient::AgentClient(const rclcpp::NodeOptions & options)
// : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
// {
//   node_->declare_parameter<std::string>("hand_planner", "interpolation");
//   node_->declare_parameter<std::string>("arm_planner", "interpolation");
//   RCLCPP_WARN(
//     LOGGER, "hand, arm planner: %s, %s ", node_->get_parameter("hand_planner").as_string().c_str(),
//     node_->get_parameter("arm_planner").as_string().c_str() );

//   hand_planner_ = get_planner(node_->get_parameter("hand_planner").as_string(), node_);

//   arm_planner_ = get_planner(node_->get_parameter("arm_planner").as_string(), node_);

// }

// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr AgentClient::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// mtc::Task AgentClient::plan()
// {
//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_, "robot_description");

//   const auto & arm_group_name = "interbotix_arm";
//   const auto & hand_group_name = "interbotix_gripper";
//   const auto & hand_frame = "locobot_fingers_link";


//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", hand_group_name);
//   task.setProperty("ik_frame", hand_frame);

// // Disable warnings for this line, as it's a variable that's set but not used in this example
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage * current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
// #pragma GCC diagnostic pop


// //   auto allow_collisions =
// //       std::make_unique<mtc::stages::ModifyPlannqingScene>("allow collisions");
// //       allow_collisions->
// //   allow_collisions->allowCollisions("object",
// //                         task.getRobotModel()
// //                             ->getJointModelGroup(hand_group_name)
// //                             ->getLinkModelNamesWithCollisionGeometry(),
// //                         true);
// // task.add(std::move(allow_collisions));


//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(1.0);
//   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//   cartesian_planner->setStepSize(.02);

//   auto move_hand =
//     std::make_unique<mtc::stages::MoveTo>("move_gripper", interpolation_planner);
//   move_hand->setGroup(hand_group_name);
//   move_hand->setGoal("Released");
//   task.add(std::move(move_hand));

//   geometry_msgs::msg::TwistStamped pose_delta;
// // Set the header
//   pose_delta.header.stamp = rclcpp::Clock().now();
//   pose_delta.header.frame_id = hand_frame; // or any other frame

// // Set the linear velocity
// // pose_delta.twist.linear.x = 0.05; // velocity in x
//   pose_delta.twist.linear.y = 0.0; // velocity in y
//   pose_delta.twist.linear.z = 0.05; // velocity in z

// // Set the angular velocity
//   pose_delta.twist.angular.x = 0.0; // angular velocity around x
//   pose_delta.twist.angular.y = 0.0; // angular velocity around y
//   pose_delta.twist.angular.z = 0.0; // angular velocity around z

//   auto move_arm =
//     std::make_unique<mtc::stages::MoveRelative>("move_arm", cartesian_planner);
//   move_arm->setGroup(arm_group_name);
//   move_arm->setMinMaxDistance(-0.07, 0.07);
//   move_arm->setDirection(pose_delta);
//   move_arm->setIKFrame(hand_frame);
//   task.add(std::move(move_arm));

//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

// // {
// //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
// //       "move to place",
// //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
// //                                                 { hand_group_name, interpolation_planner } });
// //   stage_move_to_place->setTimeout(5.0);
// //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
// //   task.add(std::move(stage_move_to_place));
// // }
//   return task;
// }


// void AgentClient::act()
// {
//   task_ = plan();

//   try {
//     task_.init();
//   } catch (mtc::InitStageException & e) {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(5)) {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }
//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//     return;
//   }

// }


bool move_hand(
  moveit::planning_interface::MoveGroupInterface & move_group_interface,
  rclcpp::Logger logger, const dgl_ros_interfaces::action::VLA::Feedback & vla)
{
  auto current_openness =
    move_group_interface.getCurrentState()->getVariablePosition("left_finger");
  RCLCPP_WARN(
    logger,
    "Current openness %f", current_openness);

  double finger_openness_target = std::clamp(
    current_openness + vla.gripper_closedness_action, 0.0195, 0.037);

  RCLCPP_WARN(
    logger,
    " openness target %f", finger_openness_target);

  move_group_interface.setJointValueTarget("left_finger", finger_openness_target);
  move_group_interface.setJointValueTarget("right_finger", -finger_openness_target);
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    move_group_interface.execute(plan);
    return success;
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  return false;

}

bool move_arm(
  moveit::planning_interface::MoveGroupInterface & move_group_interface,
  rclcpp::Logger logger, const dgl_ros_interfaces::action::VLA::Feedback & vla)
{
  // print feedback
  RCLCPP_WARN(
    logger,
    "^^^^^^^Feedback in move arm: %f %f %f", vla.world_vector[0], vla.world_vector[1], vla.world_vector[2]); 

  auto current_pose = move_group_interface.getCurrentPose("locobot_fingers_link");
  auto current_rpy = move_group_interface.getCurrentRPY("locobot_fingers_link");


  RCLCPP_WARN(
    logger,
    "Current Pose: Position - X: %f, Y: %f, Z: %f, Orientation - W: %f, X: %f, Y: %f, Z: %f",
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
    current_pose.pose.orientation.w, current_pose.pose.orientation.x,
    current_pose.pose.orientation.y, current_pose.pose.orientation.z);

  RCLCPP_WARN(
    logger, "Current RPY: Roll: %f, Pitch: %f, Yaw: %f",
    current_rpy[0], current_rpy[1], current_rpy[2]);


  // Set a target Pose
  auto const target_pose = [&vla, &current_pose, &current_rpy, &logger] {

  double factor = 3.0;
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.header.stamp = current_pose.header.stamp;
      msg.pose = poseFromXYZRPY(
        {current_pose.pose.position.x + factor * vla.world_vector[0],
          current_pose.pose.position.y + factor * vla.world_vector[1],
          current_pose.pose.position.z + factor * vla.world_vector[2],
          current_rpy[0] + factor * vla.rotation_delta[0],
          current_rpy[1] + factor * vla.rotation_delta[1],
          current_rpy[2] + factor * vla.rotation_delta[2]});
      // log warn msg
      RCLCPP_WARN(
        logger,
        "Target Pose: Position - X: %f, Y: %f, Z: %f, Orientation - W: %f, X: %f, Y: %f, Z: %f",
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.pose.orientation.w, msg.pose.orientation.x,
        msg.pose.orientation.y, msg.pose.orientation.z);
      return msg;
    }();
    RCLCPP_WARN(logger, "****Actual target pose: %f %f %f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
  move_group_interface.setPoseTarget(target_pose);
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    move_group_interface.execute(plan);
    return success;
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  return false;
}


bool act(
  moveit::planning_interface::MoveGroupInterface & move_hand_group,
  moveit::planning_interface::MoveGroupInterface & move_arm_group,
  rclcpp::Node::SharedPtr & node, const dgl_ros_interfaces::action::VLA::Feedback & vla)
{
  if (!move_hand(move_hand_group, node->get_logger(), vla)) {
    return false;
  }

  return move_arm(move_arm_group, node->get_logger(), vla);
}


int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto client = rclcpp_action::create_client<dgl_ros_interfaces::action::VLA>(
    node,
    "sample_grasp_poses");
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

    // Now, let's define a collision object ROS message for the robot to avoid.
  // moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = "world";

//   // The id of the object is used to identify it.
//   collision_object.id = "box";

//   // Define a box to add to the world.
//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 2.0;
//   primitive.dimensions[primitive.BOX_Y] = 2.0;
//   primitive.dimensions[primitive.BOX_Z] = 2.0;

//   // Define a pose for the box (specified relative to frame_id).
//   geometry_msgs::msg::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 1.2;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.0;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);


    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   RCLCPP_INFO(node->get_logger(), "Add an object into the world");
moveit::planning_interface::PlanningSceneInterface psi;
// psi.addCollisionObjects(collision_objects);

 // Allow collisions
 

  // planning_scene_interface->allowCollisions("box", "interbotix_gripper", true);
  // Create a PlanningSceneMonitor, which will provide updates to the RobotState
  // (also known as the current state) and the current scene (attached objects,
  // collison objects, etc.).

  // Allow collisions in the planning scene between the box and all links in a robot model.
//   for (const auto & link_name : psm->getRobotModel()->getLinkModelNamesWithCollisionGeometry()) {
//     psm->getPlanningScene()->getAllowedCollisionMatrixNonConst().setEntry("box", link_name, true);
//   }

// // Apply the current state to the planning scene.
//   psm->getPlanningScene()->getCurrentStateNonConst().update();
//   psm->startStateMonitor();
  using moveit::planning_interface::MoveGroupInterface;
  using dgl_ros_interfaces::action::VLA;
  using GoalHandleSharedPtr = std::shared_ptr<rclcpp_action::ClientGoalHandle<VLA>>;
  std::unique_ptr<VLA::Feedback>  prev_feedback = std::make_unique<VLA::Feedback>();;
  for (int i = 0; i < 2; ++i) {
   // Clear octomap
    psm->clearOctomap();
    auto tic = std::chrono::steady_clock::now();
    auto move_hand_group = MoveGroupInterface(node, "interbotix_gripper");
    // log warn collision objects

    auto move_arm_group = MoveGroupInterface(node, "interbotix_arm");
    std::unique_ptr<VLA::Feedback> feedback = nullptr;
    auto goal = VLA::Goal();
    goal.instructions = "pick block";

    auto options = rclcpp_action::Client<VLA>::SendGoalOptions();
    options.feedback_callback = [&feedback, &i, &prev_feedback](const GoalHandleSharedPtr gh,
        const std::shared_ptr<const VLA::Feedback> fb) {
        feedback = std::make_unique<VLA::Feedback>();
        // print feedback
        RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"), "Feedback: %f %f %f",fb->world_vector[0], fb->world_vector[1], fb->world_vector[2]); 
        *feedback = *fb;
        *prev_feedback = *fb;
      };
    // auto future = client->action_server_is_ready()
    auto future = client->async_send_goal(goal, options);

    auto toc = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic);
    // warn rclcpp ok
    RCLCPP_ERROR(node->get_logger(), "rclcpp ok = %i", rclcpp::ok());
    // warn feedback == nullptr
    RCLCPP_ERROR(node->get_logger(), "feedback == nullptr = %i", feedback == nullptr);
    // warn duration_ms < std::chrono::seconds(10)
    RCLCPP_ERROR(
      node->get_logger(), "duration_ms < std::chrono::seconds(20) = %i", duration_ms < std::chrono::seconds(
        20));
    toc = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (feedback == nullptr && duration_ms < std::chrono::seconds(20))) {
      //  RCLCPP_ERROR(node->get_logger(), "No feedback received");
      toc = std::chrono::steady_clock::now();
      duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic);
    }
    RCLCPP_WARN(
      node->get_logger(), "-----------step %i duration for action client = %i ms", i,
      duration_ms);
    if (feedback == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "No feedback received");
      feedback = std::make_unique<VLA::Feedback>();
      if (prev_feedback != nullptr) {
              RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"), "prev Feedback: %f %f %f",prev_feedback->world_vector[0], prev_feedback->world_vector[1], prev_feedback->world_vector[2]); 
           *feedback = *prev_feedback;

      }
   
    }

    bool success = act(move_hand_group, move_arm_group, node, *feedback);
    toc = std::chrono::steady_clock::now();
    duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic);
    RCLCPP_WARN(node->get_logger(), "ITER: ******* %i, duration = %i ms", i, duration_ms);
    // rclcpp::sleep_for(std::chrono::milliseconds(500));
  }


  // //   We will start by instantiating a
  // // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
  // // object, which will look up the robot description on the ROS
  // // parameter server and construct a
  // // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
  // // for us to use.
  // robot_model_loader::RobotModelLoaderPtr robot_model_loader(
  //     new robot_model_loader::RobotModelLoader(move_group_node, "robot_description"));

  // // Using the RobotModelLoader, we can construct a planning scene monitor that
  // // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
  // // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
  // // startStateMonitor to fully initialize the planning scene monitor
  // planning_scene_monitor::PlanningSceneMonitorPtr psm(
  //     new planning_scene_monitor::PlanningSceneMonitor(move_group_node, robot_model_loader));

  // /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
  //                      the internal planning scene accordingly */
  // psm->startSceneMonitor();
  // /* listens to changes of world geometry, collision objects, and (optionally) octomaps
  //                               world geometry, collision objects and optionally octomaps */
  // psm->startWorldGeometryMonitor();
  // /* listen to joint state updates as well as changes in attached collision objects
  //                       and update the internal planning scene accordingly*/
  // psm->startStateMonitor();

  // /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
  // moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
  //    for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
  //    RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
  // moveit::core::RobotStatePtr robot_state(
  //     new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  // auto agent_client = std::make_shared<AgentClient>(options);
  // rclcpp::executors::MultiThreadedExecutor executor;

  // auto spin_thread = std::make_unique<std::thread>(
  //   [&executor, &node]() {
  //     executor.add_node(node);
  //     executor.spin();
  //     executor.remove_node(node);
  //   });

  // agent_client->act();
  rclcpp::shutdown();
  return 0;
}


// #include <rclcpp/rclcpp.hpp>

// // MTC demo implementation
// #include <deep_grasp_task/deep_pick_place_task.h>

// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <iostream>

// #include <geometric_shapes/shape_operations.h>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <deep_grasp_msgs/action/cylinder_segment.hpp>
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "tf2_eigen/tf2_eigen.hpp"
// #include <eigen3/Eigen/Eigen>

// #include <deep_grasp_task/deep_pick_place_task.h>
// #include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
// #include <moveit/task_constructor/solvers/pipeline_planner.h>
// #include <deep_grasp_task/stages/deep_grasp_pose.h>
// #include <iostream>
// #include <Eigen/Dense>


// geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose)
// {
//   geometry_msgs::msg::Pose p;
//   p.position.x = pose[0];
//   p.position.y = pose[1];
//   p.position.z = pose[2];
//   Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
//                          Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
//                          Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
//   p.orientation.x = q.x();
//   p.orientation.y = q.y();
//   p.orientation.z = q.z();
//   p.orientation.w = q.w();
//   return p;
// }


//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2013, SRI International
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of SRI International nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *********************************************************************/

// /* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// // All source files that use ROS logging should define a file-specific
// // static const rclcpp::Logger named LOGGER, located at the top of the file
// // and inside the namespace with the narrowest scope (if there is one)
// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

//   // We spin up a SingleThreadedExecutor for the current state monitor to get information
//   // about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   //
//   // Setup
//   // ^^^^^
//   //
//   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
//   // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
//   // are used interchangeably.
//   static const std::string PLANNING_GROUP = "interbotix_gripper";


//   // We will start by instantiating a
//   // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
//   // object, which will look up the robot description on the ROS
//   // parameter server and construct a
//   // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
//   // for us to use.
//   // robot_model_loader::RobotModelLoaderPtr robot_model_loader(
//   //     new robot_model_loader::RobotModelLoader(move_group_node, "robot_description"));

//   // // Using the RobotModelLoader, we can construct a planning scene monitor that
//   // // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
//   // // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
//   // // startStateMonitor to fully initialize the planning scene monitor
//   // planning_scene_monitor::PlanningSceneMonitorPtr psm(
//   //     new planning_scene_monitor::PlanningSceneMonitor(move_group_node, robot_model_loader));

//   // /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
//   //                      the internal planning scene accordingly */
//   // psm->startSceneMonitor();
//   // /* listens to changes of world geometry, collision objects, and (optionally) octomaps
//   //                               world geometry, collision objects and optionally octomaps */
//   // psm->startWorldGeometryMonitor();
//   // /* listen to joint state updates as well as changes in attached collision objects
//   //                       and update the internal planning scene accordingly*/
//   // psm->startStateMonitor();

//   // /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
//   // moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

//   // /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
//   //    for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
//   //    RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
//   // moveit::core::RobotStatePtr robot_state(
//   //     new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

//   // /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
//   //    group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
//   // const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("interbotix_gripper");


//   // The
//   // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
//   // class can be easily set up using just the name of the planning group you would like to control and plan for.
//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   // // We will use the
//   // // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
//   // // class to add and remove collision objects in our "virtual world" scene
//   // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // Raw pointers are frequently used to refer to the planning group for improved performance.
//   // const moveit::core::JointModelGroup* joint_model_group =
//   //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   // namespace rvt = rviz_visual_tools;
//   // moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node);
//   // visual_tools.deleteAllMarkers();

//   /* Remote control is an introspection tool that allows users to step through a high level script */
//   /* via buttons and keyboard shortcuts in RViz */
//   // visual_tools.loadRemoteControl();

//   // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   // text_pose.translation().z() = 1.0;
//   // visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

//   // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   // visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // We can print the name of the reference frame for this robot.
//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // .. _move_group_interface-planning-to-pose-goal:
//   //
//   // Planning to a Pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // We can plan a motion for this group to a desired pose for the
//   // end-effector.
//   geometry_msgs::msg::Pose target_pose1;
//   target_pose1.orientation.w = 1.0;
//   target_pose1.position.x = 0.28;
//   target_pose1.position.y = -0.2;
//   target_pose1.position.z = 0.5;
//   move_group.setPoseTarget(target_pose1);

//   // Now, we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group
//   // to actually move the robot.
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//    // Visualizing plans
//   // ^^^^^^^^^^^^^^^^^
//   // We can also visualize the plan as a line with markers in RViz.
//   // RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
//   // visual_tools.publishAxisLabeled(target_pose1, "pose1");
//   // visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
//   // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   // visual_tools.trigger();


//   if (success) {
//     auto executed = move_group.move();
//     //  bool executed = (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   RCLCPP_INFO(LOGGER, "Excuting %s", executed == moveit::core::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");
//   }


// }
// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

// Moving to a pose goal
// ^^^^^^^^^^^^^^^^^^^^^

// Moving to a pose goal is similar to the step above
// except we now use the ``move()`` function. Note that
// the pose goal we had set earlier is still active
// and so the robot will try to move to that goal. We will
// not use that function in this tutorial since it is
// a blocking function and requires a controller to be active
// and report success on execution of a trajectory.

/* Uncomment below line when working with a real robot */
/* move_group.move(); */

// Planning to a joint-space goal
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Let's set a joint space goal and move towards it.  This will replace the
// pose target we set above.
//
// To start, we'll create an pointer that references the current robot's state.
// RobotState is the object that contains all the current position/velocity/acceleration data.
// moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
// //
// // Next get the current set of joint values for the group.
// std::vector<double> joint_group_positions;
// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

// Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
// joint_group_positions[0] = -1.0;  // radians
// bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
// if (!within_bounds)
// {
//   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
// }

// We lower the allowed maximum velocity and acceleration to 5% of their maximum.
// The default values are 10% (0.1).
// Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
// or set explicit factors in your code if you need your robot to move faster.
// move_group.setMaxVelocityScalingFactor(0.05);
// move_group.setMaxAccelerationScalingFactor(0.05);

// success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
// RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz:
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Planning with Path Constraints
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Path constraints can easily be specified for a link on the robot.
//   // Let's specify a path constraint and a pose goal for our group.
//   // First define the path constraint.
//   moveit_msgs::msg::OrientationConstraint ocm;
//   ocm.link_name = "panda_link7";
//   ocm.header.frame_id = "panda_link0";
//   ocm.orientation.w = 1.0;
//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.1;
//   ocm.weight = 1.0;

//   // Now, set it as the path constraint for the group.
//   moveit_msgs::msg::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group.setPathConstraints(test_constraints);

//   // Enforce Planning in Joint Space
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Depending on the planning problem MoveIt chooses between
//   // ``joint space`` and ``cartesian space`` for problem representation.
//   // Setting the group parameter ``enforce_joint_model_state_space:true`` in
//   // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
//   //
//   // By default, planning requests with orientation path constraints
//   // are sampled in ``cartesian space`` so that invoking IK serves as a
//   // generative sampler.
//   //
//   // By enforcing ``joint space``, the planning process will use rejection
//   // sampling to find valid requests. Please note that this might
//   // increase planning time considerably.
//   //
//   // We will reuse the old goal that we had and plan to it.
//   // Note that this will only work if the current state already
//   // satisfies the path constraints. So we need to set the start
//   // state to a new pose.
//   moveit::core::RobotState start_state(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose start_pose2;
//   start_pose2.orientation.w = 1.0;
//   start_pose2.position.x = 0.55;
//   start_pose2.position.y = -0.05;
//   start_pose2.position.z = 0.8;
//   start_state.setFromIK(joint_model_group, start_pose2);
//   move_group.setStartState(start_state);

//   // Now, we will plan to the earlier pose target from the new
//   // start state that we just created.
//   move_group.setPoseTarget(target_pose1);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group.setPlanningTime(10.0);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz:
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishAxisLabeled(start_pose2, "start");
//   visual_tools.publishAxisLabeled(target_pose1, "goal");
//   visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // When done with the path constraint, be sure to clear it.
//   move_group.clearPathConstraints();

//   // Cartesian Paths
//   // ^^^^^^^^^^^^^^^
//   // You can plan a Cartesian path directly by specifying a list of waypoints
//   // for the end-effector to go through. Note that we are starting
//   // from the new start state above.  The initial pose (start state) does not
//   // need to be added to the waypoint list but adding it can help with visualizations
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   waypoints.push_back(start_pose2);

//   geometry_msgs::msg::Pose target_pose3 = start_pose2;

//   target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // down

//   target_pose3.position.y -= 0.2;
//   waypoints.push_back(target_pose3);  // right

//   target_pose3.position.z += 0.2;
//   target_pose3.position.y += 0.2;
//   target_pose3.position.x -= 0.2;
//   waypoints.push_back(target_pose3);  // up and left

//   // We want the Cartesian path to be interpolated at a resolution of 1 cm
//   // which is why we will specify 0.01 as the max step in Cartesian
//   // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//   // Warning - disabling the jump threshold while operating real hardware can cause
//   // large unpredictable motions of redundant joints and could be a safety issue
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Cartesian motions should often be slow, e.g. when approaching objects. The speed of Cartesian
//   // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
//   // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
//   // Pull requests are welcome.
//   //
//   // You can execute a trajectory like this.
//   /* move_group.execute(trajectory); */

//   // Adding objects to the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // First, let's plan to another simple goal with no objects in the way.
//   move_group.setStartState(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose another_pose;
//   another_pose.orientation.w = 0;
//   another_pose.orientation.x = -1.0;
//   another_pose.position.x = 0.7;
//   another_pose.position.y = 0.0;
//   another_pose.position.z = 0.59;
//   move_group.setPoseTarget(another_pose);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishAxisLabeled(another_pose, "goal");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_clear_path.gif
//   //    :alt: animation showing the arm moving relatively straight toward the goal
//   //
//   // Now, let's define a collision object ROS message for the robot to avoid.
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 0.1;
//   primitive.dimensions[primitive.BOX_Y] = 1.5;
//   primitive.dimensions[primitive.BOX_Z] = 0.5;

//   // Define a pose for the box (specified relative to frame_id).
//   geometry_msgs::msg::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.48;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.25;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   RCLCPP_INFO(LOGGER, "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now, when we plan a trajectory it will avoid the obstacle.
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach an object to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between objects as well.
//   moveit_msgs::msg::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::msg::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper.
//   object_to_attach.header.frame_id = move_group.getEndEffectorLink();
//   geometry_msgs::msg::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector).
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   RCLCPP_INFO(LOGGER, "Attach the object to the robot");
//   std::vector<std::string> touch_links;
//   touch_links.push_back("panda_rightfinger");
//   touch_links.push_back("panda_leftfinger");
//   move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

//   visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group.setStartStateToCurrentState();
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   RCLCPP_INFO(LOGGER, "Detach the object from the robot");
//   move_group.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   RCLCPP_INFO(LOGGER, "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

//   // END_TUTORIAL
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   rclcpp::shutdown();
//   return 0;
// }


// #include <pluginlib/class_loader.hpp>

// // MoveIt
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/conversions.h>
// #include <moveit/planning_pipeline/planning_pipeline.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_msgs/msg/planning_scene.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.parameter_overrides({{"default_planning_plugin", "ompl"},
//   {"planning_plugin", "ompl"},{"move_group/default_planning_plugin", "ompl"}});
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto node = rclcpp::Node::make_shared("motion_planning_pipeline_tutorial", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   // Start
//   // ^^^^^
//   // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
//   // a RobotModel and a PlanningScene.
//   //
//   // We will start by instantiating a
//   // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
//   // object, which will look up the robot description on the ROS
//   // parameter server and construct a
//   // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
//   // for us to use.
//   robot_model_loader::RobotModelLoaderPtr robot_model_loader(
//       new robot_model_loader::RobotModelLoader(node, "robot_description"));

//   // Using the RobotModelLoader, we can construct a planning scene monitor that
//   // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
//   // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
//   // startStateMonitor to fully initialize the planning scene monitor
//   planning_scene_monitor::PlanningSceneMonitorPtr psm(
//       new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));

//   /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
//                        the internal planning scene accordingly */
//   psm->startSceneMonitor();
//   /* listens to changes of world geometry, collision objects, and (optionally) octomaps
//                                 world geometry, collision objects and optionally octomaps */
//   psm->startWorldGeometryMonitor();
//   /* listen to joint state updates as well as changes in attached collision objects
//                         and update the internal planning scene accordingly*/
//   psm->startStateMonitor();

//   /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
//   moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

//   /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
//      for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
//      RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
//   moveit::core::RobotStatePtr robot_state(
//       new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

//   /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
//      group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
//   const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("interbotix_gripper");

//   // We can now setup the PlanningPipeline object, which will use the ROS parameter server
//   // to determine the set of request adapters and the planning plugin to use
//   planning_pipeline::PlanningPipelinePtr planning_pipeline(
//       new planning_pipeline::PlanningPipeline(robot_model, node,"", "ompl"));

//   // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools(node);
//   // visual_tools.deleteAllMarkers();

//   /* Remote control is an introspection tool that allows users to step through a high level script
//      via buttons and keyboard shortcuts in RViz */
//   visual_tools.loadRemoteControl();

//   /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

//   /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
//   visual_tools.trigger();


//   // Pose Goal
//   // ^^^^^^^^^
//   // We will now create a motion plan request for the right arm of the Panda
//   // specifying the desired pose of the end-effector as input.
//   planning_interface::MotionPlanRequest req;
//   req.pipeline_id = "ompl";
//   req.planner_id = "RRTConnectkConfigDefault";
//   req.allowed_planning_time = 1.0;
//   req.max_velocity_scaling_factor = 1.0;
//   req.max_acceleration_scaling_factor = 1.0;
//   planning_interface::MotionPlanResponse res;
//   geometry_msgs::msg::PoseStamped pose;
//   pose.header.frame_id = "locobot_base_footprint";
//   pose.pose.position.x = 0.3;
//   pose.pose.position.y = 0.0;
//   pose.pose.position.z = 0.75;
//   pose.pose.orientation.w = 1.0;

//   // A tolerance of 0.01 m is specified in position
//   // and 0.01 radians in orientation
//   std::vector<double> tolerance_pose(3, 0.1);
//   std::vector<double> tolerance_angle(3, 0.1);

//   // We will create the request as a constraint using a helper
//   // function available from the
//   // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
//   // package.
//   req.group_name = "interbotix_gripper";
//   moveit_msgs::msg::Constraints pose_goal =
//       kinematic_constraints::constructGoalConstraints("locobot_fingers_linkc", pose, tolerance_pose, tolerance_angle);
//   req.goal_constraints.push_back(pose_goal);

//   // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
//   // representation while planning
//   {
//     planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
//     /* Now, call the pipeline and check whether planning was successful. */
//     /* Check that the planning was successful */
//     if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
//     {
//       RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
//       rclcpp::shutdown();
//       return -1;
//     }
//   }
//   // Visualize the result
//   // ^^^^^^^^^^^^^^^^^^^^
//   rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
//       node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
//   moveit_msgs::msg::DisplayTrajectory display_trajectory;

//   /* Visualize the trajectory */
//   RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
//   moveit_msgs::msg::MotionPlanResponse pose_delta;
//   res.getMessage(pose_delta);

//   display_trajectory.trajectory_start = pose_delta.trajectory_start;
//   display_trajectory.trajectory.push_back(pose_delta.trajectory);
//   display_publisher->publish(display_trajectory);
//   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
//   visual_tools.trigger();


//   RCLCPP_INFO(LOGGER, "Done");

//   rclcpp::shutdown();
//   return 0;
// }
