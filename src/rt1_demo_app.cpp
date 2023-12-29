#include <algorithm>
#include <chrono>
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

namespace chrono = std::chrono;
using dgl_ros_interfaces::action::VLA;
using VLAGoalHandleSharedPtr = std::shared_ptr<rclcpp_action::ClientGoalHandle<VLA> >;

// C++ 20 convenience for legacy CXX compilers.
#define TIME_DIFF(tic,                                                                    \
                  toc) static_cast<int>(chrono::duration_cast<chrono::milliseconds>(toc - \
                                                                                    tic).count())
#define CLAMP(x, xmin, xmax) std::max(xmin, std::min(xmax, x));

geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double>pose)
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

double scale(double x, double x_min, double x_max)
{
  return (x - x_min) / (x_max - x_min);
}

template<typename ActionT>
class RTXAgentClient : public rclcpp::Node {
public:

  RTXAgentClient(rclcpp::NodeOptions& options)
    : rclcpp::Node("RTXAgentClientNode", options)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(this, "vla");
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("hand_frame", "locobot_fingers_link");
    this->declare_parameter("instruction", "pick block");
    this->declare_parameter("arm_group", "interbotix_arm");
    this->declare_parameter("hand_group", "interbotix_gripper");

    this->declare_parameter("left_finger_joint", "left_finger");
    this->declare_parameter("right_finger_joint", "right_finger");
    this->declare_parameter("finger_joint_max", 0.037);
    this->declare_parameter("finger_joint_min", 0.0195);

    this->declare_parameter("timeout_ms", 20000);
    this->declare_parameter("action_scale", 1.0);

    timeout_ = chrono::milliseconds(this->get_parameter("timeout_ms").as_int());
    action_client_->wait_for_action_server(timeout_);

    hand_group_ = this->get_parameter("hand_group").as_string();
    arm_group_  = this->get_parameter("arm_group").as_string();

    hand_frame_         = this->get_parameter("hand_frame").as_string();
    finger_joint_max_   = this->get_parameter("finger_joint_max").as_double();
    finger_joint_min_   = this->get_parameter("finger_joint_min").as_double();
    left_finger_joint_  = this->get_parameter("left_finger_joint").as_string();
    right_finger_joint_ = this->get_parameter("right_finger_joint").as_string();
    action_scale_       = this->get_parameter("action_scale").as_double();

    move_arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this),
      arm_group_);
    move_hand_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this),
      hand_group_);
  }

  bool              move_hand(const typename ActionT::Feedback& action);
  bool              move_arm(const typename ActionT::Feedback& action);
  bool              act(planning_scene_monitor::PlanningSceneMonitor& psm);

  const std::string get_instruction()
  {
    return this->get_parameter("instruction").as_string().c_str();
  }

private:

  rclcpp::Logger logger() {
    return this->get_logger();
  }

  chrono::milliseconds timeout_;

  std::string world_frame_;
  std::string hand_frame_;
  std::string arm_group_;
  std::string hand_group_;
  std::string left_finger_joint_;
  std::string right_finger_joint_;

  double finger_joint_max_;
  double finger_joint_min_;

  typename ActionT::Feedback last_action_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>move_hand_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>move_arm_group_;

  double action_scale_;
};

template<>
bool RTXAgentClient<VLA>::move_hand(const VLA::Feedback& action) {
  RCLCPP_ERROR(logger(), "move_hand began");
  // RCLCPP_ERROR(logger(), "action.gripper_closedness_action: %f", action.gripper_closedness_action);
  // RCLCPP_ERROR(logger(), "action world_vector: %f %f %f", action.world_vector[0], action.world_vector[1], action.world_vector[2]);
  // RCLCPP_ERROR(logger(), "action rotation_delta: %f %f %f", action.rotation_delta[0], action.rotation_delta[1], action.rotation_delta[2]);

  RCLCPP_ERROR(logger(), "finger_joint_min_: %f", finger_joint_min_);
  RCLCPP_ERROR(logger(), "finger_joint_max_: %f", finger_joint_max_);
  RCLCPP_ERROR(logger(), "left_finger_joint_: %s", left_finger_joint_.c_str());
  RCLCPP_ERROR(logger(), "right_finger_joint_: %s", right_finger_joint_.c_str());
  RCLCPP_ERROR(logger(), "move_hand_group_: %s", move_hand_group_->getName().c_str());


  move_hand_group_->setStartStateToCurrentState();
  const auto   tic                        = chrono::steady_clock::now();
  RCLCPP_ERROR(logger(), "before current_finger_joint_value");
  const double current_finger_joint_value =
    move_hand_group_->getCurrentState()->getVariablePosition(
      left_finger_joint_);
  RCLCPP_ERROR(logger(), "before scale");
  const double current_openness = scale(
    current_finger_joint_value, finger_joint_min_,
    finger_joint_max_);
  RCLCPP_ERROR(logger(), "after scale");
  RCLCPP_INFO(
    logger(), "HAND OPENNESS -- Current: %f, Target: %f", current_openness,
    action.gripper_closedness_action);

  const double target_finger_joint_value = CLAMP(
    current_openness + action.gripper_closedness_action, finger_joint_min_, finger_joint_max_);

  RCLCPP_INFO(
    logger(), "FINGER JOINT -- Current: %f, Target: %f", current_finger_joint_value,
    target_finger_joint_value);
  move_hand_group_->setJointValueTarget(left_finger_joint_,  target_finger_joint_value);
  move_hand_group_->setJointValueTarget(right_finger_joint_, -target_finger_joint_value);

  // Create a plan to the target.
  const auto [success, plan] = [this] {
                                 moveit::planning_interface::MoveGroupInterface::Plan msg;
                                 auto const ok = static_cast<bool>(move_hand_group_->plan(msg));

                                 return std::make_pair(ok, msg);
                               }();


  const auto toc = chrono::steady_clock::now();

  RCLCPP_INFO(logger(), "move_hand %i ms", TIME_DIFF(tic, toc));

  // Execute the plan
  if (success)
  {
    move_hand_group_->execute(plan);
    return success;
  }
  else
  {
    RCLCPP_ERROR(logger(), "Planning failed!");
  }
  return false;
}

template<>
bool RTXAgentClient<VLA>::move_arm(const VLA::Feedback& action) {
  move_arm_group_->setStartStateToCurrentState();
  const auto tic = chrono::steady_clock::now();

  RCLCPP_INFO(
    logger(), "VLA WORLD VECTOR in move_arm: %f %f %f", action.world_vector[0],
    action.world_vector[1], action.world_vector[2]);

  const auto current_pose = move_arm_group_->getCurrentPose(hand_frame_);
  const auto current_rpy  = move_arm_group_->getCurrentRPY(hand_frame_);

  const auto log_pose =
    [this](const std::string& name, const auto& pose, const auto& rpy) {
      RCLCPP_INFO(
        logger(),
        "%s XYZ -- X: %f, Y: %f, Z: %f, RPY --  R: %f, P: %f, Y: %f, Q -- W: %f, X: %f, Y: %f, Z: %f",
        name.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        rpy[0],
        rpy[1],
        rpy[2],
        pose.pose.orientation.w,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z);
    };

  log_pose("Current", current_pose, current_rpy);

  const auto [target_pose, target_rpy] = [&action, &current_pose, &current_rpy, this, &log_pose] {
                                          const std::vector<double> target_xyz = {
                                            current_pose.pose.position.x + action_scale_ *
                                            action.world_vector[0],
                                            current_pose.pose.position.y + action_scale_ *
                                            action.world_vector[1],
                                            current_pose.pose.position.z + action_scale_ *
                                            action.world_vector[2] };

                                           const std::vector<double> target_rpy = {
                                             current_rpy[0] + action_scale_ *
                                             action.rotation_delta[0],
                                             current_rpy[1] + action_scale_ *
                                             action.rotation_delta[1],
                                             current_rpy[2] + action_scale_ *
                                             action.rotation_delta[2] };

                                           geometry_msgs::msg::PoseStamped target;

                                           target.header.frame_id = world_frame_;
                                           target.header.stamp    = current_pose.header.stamp;

                                           target.pose = poseFromXYZRPY(
                                             { target_xyz[0], target_xyz[1], target_xyz[2],
                                               target_rpy[0], target_rpy[1],
                                               target_rpy[2] });
                                           return std::make_pair(target, target_rpy);
                                         }();

  log_pose("Target", target_pose, target_rpy);
  move_arm_group_->setPoseTarget(target_pose);

  // Create a plan to that target pose
  const auto [success, plan] = [this] {
                                 moveit::planning_interface::MoveGroupInterface::Plan msg;
                                 auto const ok = static_cast<bool>(move_arm_group_->plan(msg));

                                 return std::make_pair(ok, msg);
                               }();

  const auto toc = chrono::steady_clock::now();

  RCLCPP_INFO(logger(), "move_arm  took %i ms", TIME_DIFF(tic, toc));

  // Execute the plan
  if (success) {
    move_arm_group_->execute(plan);
    return success;
  }
  else {
    RCLCPP_ERROR(logger(), "Planning failed!");
  }
  return false;
}

template<>
bool RTXAgentClient<VLA>::act(planning_scene_monitor::PlanningSceneMonitor& psm) {
  const auto tic = chrono::steady_clock::now();

  psm.updateFrameTransforms();
  psm.clearOctomap(); // Don't worry about colliding with octomap.

  auto goal = VLA::Goal();

  goal.instructions = get_instruction();
  auto options = rclcpp_action::Client<VLA>::SendGoalOptions();

  std::shared_ptr<const VLA::Feedback> new_action = nullptr;

  options.feedback_callback = [&new_action, this](const VLAGoalHandleSharedPtr gh,
                                                const std::shared_ptr<const VLA::Feedback>fb) {
                                RCLCPP_ERROR(logger(), "Feedback callback");
                                (void)gh;
                                RCLCPP_INFO(
                                  logger(),
                                  "Feedback: %f %f %f",
                                  fb->world_vector[0],
                                  fb->world_vector[1],
                                  fb->world_vector[2]);
                                new_action = std::move(fb);
                                last_action_ = *new_action;
                              };

  action_client_->async_send_goal(goal, options);
  while ( new_action == nullptr) {
    RCLCPP_ERROR_ONCE(logger(), "Waiting...");
  }


  // RCLCPP_ERROR(logger(), "WAIT FOR NOT WORKING !! seg fault due to this callback not being called!!");
  // std::future_status status;
  //  while (status != std::future_status::ready) {
  //   status = future.wait_for(std::chrono::seconds(1));
  //   RCLCPP_ERROR(logger(), "Waiting...");
  //  }
  // if (future.wait_for(timeout_) != std::future_status::ready) {
  //   RCLCPP_ERROR(logger(), "Action server not available after timeout");
  //   return move_hand(last_action_) && move_arm(last_action_);
  // }

  const auto toc = chrono::steady_clock::now();

  RCLCPP_INFO(logger(), "Action server available after %i ms", TIME_DIFF(tic, toc));

  return move_hand(*new_action) && move_arm(*new_action);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<RTXAgentClient<VLA> >(options);

  // Spin in the background for planning scene to work.
  rclcpp::executors::MultiThreadedExecutor executor;

  /* *INDENT-OFF* */
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();
  /* *INDENT-ON* */

  planning_scene_monitor::PlanningSceneMonitor psm(node, "robot_description");

  psm.startSceneMonitor();
  psm.startWorldGeometryMonitor();

  for (int i = 0; i < 10; ++i) {
    const auto tic = chrono::steady_clock::now();
    node->act(psm);
    const auto toc = chrono::steady_clock::now();
    RCLCPP_INFO(node->get_logger(), "Action took %i ms", TIME_DIFF(tic, toc));
  }
  rclcpp::shutdown();
  return 0;
}
