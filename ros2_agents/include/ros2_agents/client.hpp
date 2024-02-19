#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ros2_agents_interfaces/action/mbodied.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace chrono = std::chrono;
namespace rvt    = rviz_visual_tools;

using ros2_agents_interfaces::action::Mbodied;
using GoalHandleSharedPtr = std::shared_ptr<rclcpp_action::ClientGoalHandle<Mbodied> >;

// C++ 20 convenience for legacy CXX compilers.
#define TIME_DIFF(tic,                                                                    \
                  toc) static_cast<int>(chrono::duration_cast<chrono::milliseconds>(toc - \
                                                                                    tic).count())
#define CLAMP(x, xmin, xmax) std::max(xmin, std::min(xmax, x))


namespace mbodied
{
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

  double clip(double x, double x_min, double x_max)
  {
    // if (x >= 0) {
    //   return CLAMP(x, x_min, x_max);
    // } else {
    //   return CLAMP(x, -x_max, -x_min);
    // }
    if (x >= 0)
    {
      if (x > x_min)
      {
        return CLAMP(x, 0.0, x_max);
      }
      else
      {
        return 0.0;
      }
    }
    else
    {
      if (x < -x_min)
      {
        return CLAMP(x, -x_max, 0.0);
      }
      else
      {
        return 0.0;
      }
    }
  }

  template<typename ActionT>
  class AgentClient : public rclcpp::Node {
public:

    AgentClient(rclcpp::NodeOptions& options)
      : rclcpp::Node("AgentClientNode", options)
    {
      action_client_ = rclcpp_action::create_client<ActionT>(this, "mbodied");
      this->declare_parameter("world_frame", "world");
      this->declare_parameter("hand_frame", "locobot_fingers_link");
      this->declare_parameter("num_iterations", 100);
      this->declare_parameter("instruction", "pick block");
      this->declare_parameter("arm_group", "interbotix_arm");
      this->declare_parameter("hand_group", "interbotix_gripper");

      this->declare_parameter("left_finger_joint", "left_finger");
      this->declare_parameter("right_finger_joint", "right_finger");
      this->declare_parameter("finger_joint_max", 0.037);
      this->declare_parameter("finger_joint_min", 0.0);

      this->declare_parameter("timeout_ms", 20000);
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

      this->declare_parameter("xyz_bound_lo", 0.0);
      this->declare_parameter("xyz_bound_hi", 0.25);
      this->declare_parameter("rpy_bound_lo", 0.03);
      this->declare_parameter("rpy_bound_hi", 6.28);

      this->declare_parameter("cartesian", false);
      this->declare_parameter("cartesian_step_size", 0.0002);
      this->declare_parameter("xyz_tolerance", 0.01);
      this->declare_parameter("rpy_tolerance", 0.1);
      this->declare_parameter("xyz_scale", 35.0);
      this->declare_parameter("rpy_scale", 6.0);
    }

    void init()
    {
      move_arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this),
        arm_group_);
      move_arm_group_->setEndEffectorLink(hand_frame_);

      visual_tools_ =  std::make_unique<
        moveit_visual_tools::MoveItVisualTools>(
        std::shared_ptr<rclcpp::Node>(this),
        world_frame_);


      visual_tools_->deleteAllMarkers();

      move_arm_group_->startStateMonitor();

      // move_arm_group_->setMaxAccelerationScalingFactor(0.05);
      // move_arm_group_->setMaxVelocityScalingFactor(0.05);
      move_arm_group_->setPoseReferenceFrame(world_frame_);

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
        RCLCPP_INFO(logger(), "Action SUCCEEDED (result): %s",
                    result.result->grasp_state.c_str());
      }
      else
      {
        RCLCPP_ERROR(logger(), "Action error %s",
                     result.result->grasp_state.c_str());
      }
    }

private:

    rclcpp::Logger logger()
    {
      return this->get_logger();
    }

    chrono::milliseconds timeout_;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools>visual_tools_;
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
  void AgentClient<Mbodied>::feedback_callback(
    rclcpp_action::ClientGoalHandle<Mbodied>::SharedPtr gh,
    const std::shared_ptr<const Mbodied::Feedback>      fb)
  {
    (void)gh;
    RCLCPP_INFO(
      logger(),
      "Feedback WORLD VECTOR: %f %f %f",
      fb->actions[0].left_hand.x,
     fb->actions[0].left_hand.y,
     fb->actions[0].left_hand.z);


    last_action_ = *fb;
    action_ready = true;
  }

  // TODO(speralta): Make more general by passing in current openness ->
  // joint_names + values function
  template<>
  bool AgentClient<Mbodied>::move_hand(const Mbodied::Feedback& action)
  {
    const auto tic = chrono::steady_clock::now();

    const double current_finger_joint_value =
      move_hand_group_->getCurrentState()->getVariablePosition(
        left_finger_joint_);

    const double current_right_finger_joint_value =
      move_hand_group_->getCurrentState()->getVariablePosition(
        right_finger_joint_);

    const double current_openness = scale(
      current_finger_joint_value, finger_joint_min_,
      finger_joint_max_);

    // const double target_openness = 0.5 * (action.gripper_closedness_action +
    // 1);
    const double target_openness =
      current_openness + action.actions[0].left_hand.grasp;

    RCLCPP_INFO(
      logger(), "HAND OPENNESS -- Current: %f, Target: %f", current_openness,
      target_openness);

    const double target_finger_joint_value = unscale(
      target_openness, finger_joint_min_,
      finger_joint_max_);



    RCLCPP_INFO(
      logger(),
      "FINGER JOINT -- Current: %f, %f, Target: %f, %f",
      current_finger_joint_value,
      current_right_finger_joint_value,
      target_finger_joint_value,
      -target_finger_joint_value);

    move_hand_group_->setJointValueTarget(left_finger_joint_,  target_finger_joint_value);
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
  bool AgentClient<Mbodied>::move_arm(const Mbodied::Feedback& action)
  {
    move_arm_group_->setGoalOrientationTolerance(get_param<double>("rpy_tolerance"));
    move_arm_group_->setGoalPositionTolerance(get_param<double>("xyz_tolerance"));
    move_arm_group_->setGoalJointTolerance(get_param<double>("rpy_tolerance"));
    move_arm_group_->setPlanningTime(10.0);
    const auto tic = chrono::steady_clock::now();

    RCLCPP_INFO(
      logger(), "Mbodied xyz in move_arm: %f %f %f", action.actions[0].left_hand.x,
       action.actions[0].left_hand.y,  action.actions[0].left_hand.z);

    const auto current_pose = move_arm_group_->getCurrentPose(hand_frame_).pose;
    const auto current_rpy  = move_arm_group_->getCurrentRPY(hand_frame_);

    const auto log_pose =
      [this](const std::string& name, const auto& pose, const auto& rpy) {
        RCLCPP_INFO(
          logger(),
          "%s  XYZ -- X: %f, Y: %f, Z: %f, RPY --  R: %f, P: %f, Y: %f, Q -- W: %f, X: %f, Y: %f, Z: %f",
          name.c_str(),
          pose.position.x,
          pose.position.y,
          pose.position.z,
          rpy[0],
          rpy[1],
          rpy[2],
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);
      };

    log_pose("Current", current_pose, current_rpy);

    auto xyz_bound_hi = get_param<double>("xyz_bound_hi");
    auto xyz_bound_lo = get_param<double>("xyz_bound_lo");
    auto rpy_bound_hi = get_param<double>("rpy_bound_hi");
    auto rpy_bound_lo = get_param<double>("rpy_bound_lo");

    auto xyz_scale = get_param<double>("xyz_scale");
    auto rpy_scale = get_param<double>("rpy_scale");

    const auto [target_pose,
                target_rpy] = [&action, &current_pose, &current_rpy, &rpy_scale, xyz_scale,
                               &xyz_bound_lo, &xyz_bound_hi, &rpy_bound_lo, &rpy_bound_hi, this] {
                                const std::vector<double> target_xyz = {
                                  current_pose.position.x + clip(xyz_scale *
                                                                  action.actions[0].left_hand.x,
                                                                 xyz_bound_lo,
                                                                 xyz_bound_hi),
                                  current_pose.position.y + clip(xyz_scale *
                                                                action.actions[0].left_hand.y,
                                                                 xyz_bound_lo,
                                                                 xyz_bound_hi),
                                  current_pose.position.z + clip(xyz_scale *
                                                                action.actions[0].right_hand.z,
                                                                 xyz_bound_lo,
                                                                 xyz_bound_hi) };

                                const std::vector<double> target_rpy = {
                                  current_rpy[0] + clip(rpy_scale *
                                                 action.actions[0].left_hand.roll,
                                                        rpy_bound_lo,
                                                        rpy_bound_hi),
                                  current_rpy[1] + clip(rpy_scale *
                                                    action.actions[0].left_hand.pitch,
                                                        rpy_bound_lo,
                                                        rpy_bound_hi),
                                  current_rpy[2] + clip(rpy_scale *
                                                      action.actions[0].left_hand.yaw,
                                                        rpy_bound_lo,
                                                        rpy_bound_hi) };

                                geometry_msgs::msg::Pose target = poseFromXYZRPY(
                                  { target_xyz[0], target_xyz[1], target_xyz[2],
                                    target_rpy[0], target_rpy[1],
                                    target_rpy[2] });

                                return std::make_pair(target, target_rpy);
                              }();

    log_pose("Target", target_pose, target_rpy);


    // Create a plan to that target pose
    const   moveit::core::MoveItErrorCode result = [this, &target_pose] {
                                                     const auto tic = chrono::steady_clock::now();
                                                     moveit::core::MoveItErrorCode result;

                                                     if (!get_param<bool>("cartesian"))
                                                     {
                                                       move_arm_group_->setPoseTarget(target_pose);


                                                       move_arm_group_->move();
                                                     }
                                                     else
                                                     {
                                                       moveit_msgs::msg::RobotTrajectory trajectory;
                                                       std::vector<geometry_msgs::msg::Pose>
                                                       waypoints;

                                                       waypoints.push_back(target_pose);

                                                       visual_tools_->deleteAllMarkers();
                                                       Eigen::Isometry3d text_pose =
                                                         Eigen::Isometry3d::Identity();
                                                       visual_tools_->publishText(text_pose,
                                                                                  "Cartesian_Path",
                                                                                  rvt::WHITE,
                                                                                  rvt::XLARGE);
                                                       visual_tools_->publishPath(waypoints,
                                                                                  rvt::LIME_GREEN,
                                                                                  rvt::SMALL);

                                                       for (std::size_t i = 0; i < waypoints.size();
                                                            ++i)
                                                       {
                                                         visual_tools_->publishAxisLabeled(
                                                           waypoints[i],
                                                           "pt" + std::to_string(i),
                                                           rvt::SMALL);
                                                       }

                                                       const float distance =
                                                         move_arm_group_->computeCartesianPath(
                                                           waypoints,
                                                           get_param<double>(
                                                             "cartesian_step_size"),
                                                           5.0,
                                                           trajectory,
                                                           get_param<bool>(
                                                             "avoid_collisions"));
                                                       RCLCPP_INFO(
                                                         logger(),
                                                         "move_arm planned distance reached %f ",
                                                         distance);

                                                       const moveit::core::JointModelGroup *
                                                         joint_model_group =
                                                         move_arm_group_->getCurrentState()->
                                                         getJointModelGroup(arm_group_);
                                                       visual_tools_->publishTrajectoryLine(
                                                         trajectory,
                                                         joint_model_group);
                                                       visual_tools_->trigger();
                                                       visual_tools_->publishTrajectoryPath(
                                                         trajectory,
                                                         move_arm_group_->getCurrentState());

                                                       result =
                                                         move_arm_group_->execute(trajectory);
                                                     }


                                                     const auto toc = chrono::steady_clock::now();

                                                     RCLCPP_INFO(logger(),
                                                                 "move_arm planned in %i ms",
                                                                 TIME_DIFF(tic, toc));
                                                     return result;
                                                   }();


    // Execute the plan
    // moveit::core::MoveItErrorCode result_status =
    // moveit::core::MoveItErrorCode::FAILURE;

    // if (success)
    // {
    //   result_status = move_arm_group_->execute(plan);
    // }

    const auto toc = chrono::steady_clock::now();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger(),
                  "move_arm status: %s in %i ms ",
                  moveit::core::error_code_to_string(result).c_str(),
                  TIME_DIFF(tic, toc));
      return true;
    }
    else
    {
      RCLCPP_ERROR(logger(),
                   "move_arm status: %s in %i ms ",
                   moveit::core::error_code_to_string(result).c_str(),
                   TIME_DIFF(tic, toc));
      return false;
    }
  }

  template<>
  bool AgentClient<Mbodied>::act()
  {
    const auto tic = chrono::steady_clock::now();

    // psm.updateFrameTransforms();
    // psm.clearOctomap(); // Don't worry about colliding with octomap.

    auto goal = Mbodied::Goal();

    goal.instruction = get_param<std::string>("instruction");
    auto options = rclcpp_action::Client<Mbodied>::SendGoalOptions();

    options.goal_response_callback = std::bind(&AgentClient<Mbodied>::goal_response_callback,
                                               this,
                                               std::placeholders::_1);
    options.feedback_callback = std::bind(&AgentClient<Mbodied>::feedback_callback,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2);
    options.result_callback = std::bind(&AgentClient<Mbodied>::result_callback,
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
} // namespace mbodied
