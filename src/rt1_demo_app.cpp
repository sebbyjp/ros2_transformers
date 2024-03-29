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


namespace r2t {
geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double>pose)
{
  geometry_msgs::msg::Pose p;

  p.position.x = pose[0];
  p.position.y = pose[1];
  p.position.z = pose[2];
  Eigen::Quaterniond q =  Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());

  q.normalize();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

double scale(double x,
             double x_min,
             double x_max)
{
  return CLAMP((x - x_min) / (x_max - x_min), 0.0, 1.0);
}

double unscale(double x,
               double x_min,
               double x_max)
{
  return CLAMP(x * (x_max - x_min) + x_min, x_min, x_max);
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
    this->declare_parameter("num_iterations", 100);
    this->declare_parameter("instruction", "pick block");
    this->declare_parameter("arm_group", "interbotix_arm");
    this->declare_parameter("hand_group", "interbotix_gripper");

    this->declare_parameter("left_finger_joint", "left_finger");
    this->declare_parameter("right_finger_joint", "right_finger");
    this->declare_parameter("finger_joint_max", 0.037);
    this->declare_parameter("finger_joint_min", 0.015);

    this->declare_parameter("timeout_ms", 20000);
    this->declare_parameter("action_scale", 1.0);
    this->declare_parameter("avoid_collisions", true);

    // Sleep time between actions in milliseconds.
    this->declare_parameter("sleep_ms", 100);

    timeout_ = chrono::milliseconds(get_param<int>("timeout_ms"));
    action_client_->wait_for_action_server(timeout_);

    hand_group_ = this->get_parameter("hand_group").as_string();
    arm_group_  = this->get_parameter("arm_group").as_string();

    world_frame_        = this->get_parameter("world_frame").as_string();
    hand_frame_         = this->get_parameter("hand_frame").as_string();
    finger_joint_max_   = this->get_parameter("finger_joint_max").as_double();
    finger_joint_min_   = this->get_parameter("finger_joint_min").as_double();
    left_finger_joint_  = this->get_parameter("left_finger_joint").as_string();
    right_finger_joint_ = this->get_parameter("right_finger_joint").as_string();

    this->declare_parameter("translation_bound", 0.03);
    this->declare_parameter("rotation_bound", 0.25);

    
  }

  void init() {
      move_arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this),
      arm_group_);
    // move_arm_group_->setGoalOrientationTolerance(0.05);
    // move_arm_group_->setGoalPositionTolerance(0.01);
    move_arm_group_->startStateMonitor();
    // move_arm_group_->setStartStateToCurrentState();
    // move_arm_group_->setWorkspace(-1, -2.0, 0, 2.0, 2.0, 2.0);

    move_hand_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this),
      hand_group_);
    move_hand_group_->startStateMonitor();
    // move_hand_group_->setGoalJointTolerance(0.001);
    // move_hand_group_->setStartStateToCurrentState();
  }
  bool move_hand(const typename ActionT::Feedback& action);
  bool move_arm(const typename ActionT::Feedback& action);
  bool act();

  void feedback_callback(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr    gh,
    const typename std::shared_ptr<const typename ActionT::Feedback>fb);

  template<typename T>
  const T get_param(const std::string& name)
  {
    return this->get_parameter(name).get_value<T>();
  }

  void goal_response_callback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(logger(), "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(logger(), "VLA SUCCEEDED (result): %s",
                  result.result->grasp_state.c_str());
    }
    else
    {
      RCLCPP_ERROR(logger(), "VLA error %s",
                   result.result->grasp_state.c_str());
    }
  }

private:

  rclcpp::Logger logger()
  {
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

  bool action_ready = false;
  typename ActionT::Feedback last_action_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>move_hand_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>move_arm_group_;
};

template<>
void RTXAgentClient<VLA>::feedback_callback(
  rclcpp_action::ClientGoalHandle<VLA>::SharedPtr gh,
  const std::shared_ptr<const VLA::Feedback>      fb)
{
  (void)gh;
  RCLCPP_INFO(
    logger(),
    "Feedback WORLD VECTOR: %f %f %f",
    fb->world_vector[0],
    fb->world_vector[1],
    fb->world_vector[2]);
  
  auto translation_bound = get_param<double>("translation_bound");
  auto rotation_bound = get_param<double>("rotation_bound");
  VLA::Feedback action_clipped = *fb;
  action_clipped.world_vector[0] = CLAMP(action_clipped.world_vector[0], -translation_bound, translation_bound);
  action_clipped.world_vector[1] = CLAMP(action_clipped.world_vector[1], -translation_bound, translation_bound);
  action_clipped.world_vector[2] = CLAMP(action_clipped.world_vector[2], -translation_bound, translation_bound);
  action_clipped.rotation_delta[0] = CLAMP(action_clipped.rotation_delta[0], -rotation_bound, rotation_bound);
  action_clipped.rotation_delta[1] = CLAMP(action_clipped.rotation_delta[1], -rotation_bound, rotation_bound);
  action_clipped.rotation_delta[2] = CLAMP(action_clipped.rotation_delta[2], -rotation_bound, rotation_bound);
  last_action_ = action_clipped;
  action_ready = true;
}

// TODO(speralta): Make more general by passing in current openness ->
// joint_names + values function
template<>
bool RTXAgentClient<VLA>::move_hand(const VLA::Feedback& action)
{
  const auto tic = chrono::steady_clock::now();
  // move_hand_group_->setStartStateToCurrentState();
  const double current_finger_joint_value =
    move_hand_group_->getCurrentState()->getVariablePosition(
      left_finger_joint_);
  
  const double current_right_finger_joint_value =
    move_hand_group_->getCurrentState()->getVariablePosition(
      right_finger_joint_);

  const double current_openness = scale(
    current_finger_joint_value, finger_joint_min_,
    finger_joint_max_);
  // const double target_openness = 0.5 * (action.gripper_closedness_action + 1);
  const double target_openness =
    current_openness + action.gripper_closedness_action;

  RCLCPP_WARN(
    logger(), "HAND OPENNESS -- Current: %f, Target: %f", current_openness,
    target_openness);

  const double target_finger_joint_value = unscale(
    target_openness, finger_joint_min_,
    finger_joint_max_);

    //TODO(speralta): Fix unscale for right finger joint it should be 
    // -current_finger_joint_value 


  RCLCPP_WARN(
    logger(), "FINGER JOINT -- Current: %f, %f, Target: %f, %f", current_finger_joint_value, current_right_finger_joint_value,
    target_finger_joint_value, -target_finger_joint_value);


  // moveit_msgs::msg::RobotTrajectory trajectory;

  // trajectory.joint_trajectory.joint_names.push_back(left_finger_joint_);
  // trajectory.joint_trajectory.joint_names.push_back(right_finger_joint_);

  // trajectory_msgs::msg::JointTrajectoryPoint current;

  // current.positions.push_back(current_finger_joint_value);
  // current.positions.push_back(-current_right_finger_joint_value);

  // trajectory.joint_trajectory.points.push_back(current);

  // trajectory_msgs::msg::JointTrajectoryPoint target;

  // target.time_from_start = rclcpp::Duration::from_seconds(0.01);
  // target.positions.push_back(target_finger_joint_value);
  // target.positions.push_back(-target_finger_joint_value);
  // trajectory.joint_trajectory.points.push_back(target);
  // moveit::core::MoveItErrorCode result_status = move_hand_group_->execute(trajectory);

//  for (const auto& j: move_hand_group_->getActiveJoints()) {
//   RCLCPP_WARN(logger(), "active joint: %s", j.c_str());
//  }
//  for (const auto& j: move_hand_group_->getCurrentJointValues()) {
//   RCLCPP_WARN(logger(), "active joint: %f", j);
//  }
 
  move_hand_group_->setJointValueTarget(left_finger_joint_, target_finger_joint_value);
 move_hand_group_->setJointValueTarget(right_finger_joint_, -target_finger_joint_value);
  moveit::core::MoveItErrorCode result_status = move_hand_group_->move();
  const auto toc                              = chrono::steady_clock::now();

  RCLCPP_INFO(logger(),
              "move_hand status: %s took %i ms",
              moveit::core::error_code_to_string(result_status).c_str(),
              TIME_DIFF(tic, toc));
  return result_status == moveit::core::MoveItErrorCode::SUCCESS;
}

template<>
bool RTXAgentClient<VLA>::move_arm(const VLA::Feedback& action)
{
  auto action_scale = get_param<double>("action_scale");
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
        "%s (%s frame) XYZ -- X: %f, Y: %f, Z: %f, RPY --  R: %f, P: %f, Y: %f, Q -- W: %f, X: %f, Y: %f, Z: %f",
        name.c_str(),
        pose.header.frame_id.c_str(),
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

  const auto [target_pose,
              target_rpy] = [&action, &current_pose, &current_rpy, &action_scale, this] {
                              const std::vector<double> target_xyz = {
                                current_pose.pose.position.x + action_scale *
                                action.world_vector[0],
                                current_pose.pose.position.y + action_scale *
                                action.world_vector[1],
                                current_pose.pose.position.z + action_scale *
                                action.world_vector[2] };

                              const std::vector<double> target_rpy = {
                                current_rpy[0] + action_scale *
                                action.rotation_delta[0],
                                current_rpy[1] + action_scale *
                                action.rotation_delta[1],
                                current_rpy[2] + action_scale *
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


  // Create a plan to that target pose
  const auto [success, plan] = [this, &target_pose] {
          const auto tic = chrono::steady_clock::now();
      bool ok;
      moveit::planning_interface::MoveGroupInterface::Plan
                      plan;
      if (get_param<bool>("avoid_collisions")) {


                                 // move_arm_group_->setPoseTarget(target_pose);
                           
                                 // move_arm_group_->setPoseTarget(target_pose);

                                 move_arm_group_->setPoseTarget(target_pose);

                      
                                   ok  =
                                   static_cast<bool>(move_arm_group_->plan(plan));
      } else {

                                 std::vector<geometry_msgs::msg::Pose> waypoints;

                                 waypoints.push_back(target_pose.pose);
                                 const float distance = move_arm_group_->computeCartesianPath(
                                   waypoints,
                                   0.01,
                                   0.0,
                                   plan.trajectory_,
                                   get_param<bool>(
                                     "avoid_collisions"));
                                  ok = distance    > 0;
      }
                                 const auto toc = chrono::steady_clock::now();

                                 if (ok)
                                 {
                                   RCLCPP_WARN(logger(),
                                               "move_arm planned in %i ms SUCCESS",
                                               TIME_DIFF(tic, toc));
                                 }
                                 else
                                 {
                                   RCLCPP_ERROR(logger(),
                                                "move_arm planned in %i ms FAILED",
                                                TIME_DIFF(tic, toc));
                                 }

                                 return std::make_pair(ok, plan);
                               }();


  // Execute the plan
  moveit::core::MoveItErrorCode result_status = moveit::core::MoveItErrorCode::FAILURE;

  if (success)
  {
    result_status = move_arm_group_->execute(plan);
  }

  const auto toc = chrono::steady_clock::now();

  if (result_status == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger(),
                "move_arm status: %s in %i ms ",
                moveit::core::error_code_to_string(result_status).c_str(),
                TIME_DIFF(tic, toc));
    return true;
  }
  else
  {
    RCLCPP_ERROR(logger(),
                 "move_arm status: %s in %i ms ",
                 moveit::core::error_code_to_string(result_status).c_str(),
                 TIME_DIFF(tic, toc));
    return false;
  }
}

template<>
bool RTXAgentClient<VLA>::act()
{
  const auto tic = chrono::steady_clock::now();

  // psm.updateFrameTransforms();
  // psm.clearOctomap(); // Don't worry about colliding with octomap.

  auto goal = VLA::Goal();

  goal.instructions = get_param<std::string>("instruction");
  auto options = rclcpp_action::Client<VLA>::SendGoalOptions();

  options.goal_response_callback = std::bind(&RTXAgentClient<VLA>::goal_response_callback,
                                             this,
                                             std::placeholders::_1);
  options.feedback_callback = std::bind(&RTXAgentClient<VLA>::feedback_callback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2);
  options.result_callback = std::bind(&RTXAgentClient<VLA>::result_callback,
                                      this,
                                      std::placeholders::_1);
  action_client_->async_send_goal(goal, options);

  while (rclcpp::ok() && !action_ready)
  {
    RCLCPP_ERROR_ONCE(logger(), "Waiting...");
  }
  action_ready = false;

  const auto toc = chrono::steady_clock::now();

  RCLCPP_INFO(logger(), "Action server available after %i ms", TIME_DIFF(tic, toc));

  return move_hand(last_action_) && move_arm(last_action_);
}

} // namespace r2t

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto rtx_client_node = std::make_shared<r2t::RTXAgentClient<VLA> >(options);

  // Spin in the background for planning scene to work.
  rclcpp::executors::MultiThreadedExecutor executor;

  /* *INDENT-OFF* */
  executor.add_node(rtx_client_node);
  std::thread([&executor]() {executor.spin();}).detach();
 /* *INDENT-ON* */
  rtx_client_node->init();

  for (int i = 0; i < rtx_client_node->get_param<int>("num_iterations"); ++i)
  {
    if (!rclcpp::ok())
    {
      break;
    }
    RCLCPP_INFO(rtx_client_node->get_logger(), "*** Iteration %i", i);
    const auto tic = chrono::steady_clock::now();
    rtx_client_node->act();
    const auto toc = chrono::steady_clock::now();
    RCLCPP_INFO(rtx_client_node->get_logger(), "Action took %i ms", TIME_DIFF(tic, toc));
    std::this_thread::sleep_for(chrono::milliseconds(rtx_client_node->get_param<int>("sleep_ms")));
  }
  rclcpp::shutdown();
  return 0;
}
