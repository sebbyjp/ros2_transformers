/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein, Boston Cleek
   Desc:   A demo to show MoveIt Task Constructor using a deep learning based
           grasp generator
*/

// ROS
#include <rclcpp/rclcpp.hpp>

// MTC demo implementation

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <iostream>

#include <geometric_shapes/shape_operations.h>
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "tf2_eigen/tf2_eigen.hpp"
#include <eigen3/Eigen/Eigen>

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

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::msg::CollisionObject createTable(rclcpp::Node::SharedPtr node)
{
  std::string table_name, table_reference_frame;
  std::vector<double> table_dimensions;
  std::vector<double> pose;
  node->get_parameter("table_name", table_name);
  node->get_parameter("table_reference_frame", table_reference_frame);
  node->get_parameter("table_dimensions", table_dimensions);
  node->get_parameter("table_pose", pose);

  moveit_msgs::msg::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { table_dimensions[0], table_dimensions[1],
                                      table_dimensions[2] };  // align surface with world

  object.primitive_poses.push_back(poseFromXYZRPY(pose));
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  return object;
}

moveit_msgs::msg::CollisionObject createObject(rclcpp::Node::SharedPtr node, std::string name)
{
  std::string object_name, object_reference_frame;
  std::vector<double> object_dimensions;
  std::vector<double> pose;
  node->get_parameter(name + "_name", object_name);
  node->get_parameter("object_reference_frame", object_reference_frame);
  node->get_parameter(name + "_dimensions", object_dimensions);
  node->get_parameter(name + "_pose", pose);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { object_dimensions[0], object_dimensions[1], object_dimensions[2] };
  object.primitive_poses.push_back(poseFromXYZRPY(pose));
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  return object;
}

std::string getVersion() {
  if (__cplusplus == 202101L) return "C++23";
    else if (__cplusplus == 202002L) return "C++20";
    else if (__cplusplus == 201703L) return "C++17";
    else if (__cplusplus == 201402L) return "C++14";
    else if (__cplusplus == 201103L) return "C++11";
    else if (__cplusplus == 199711L) return "C++98";
    else return "pre-standard C++.";
}
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("deep_pick_place_task", node_options);
  RCLCPP_INFO(node->get_logger(), "Init deep_grasp_demo.");
  RCLCPP_INFO_STREAM(node->get_logger(), "C++ version"  << getVersion().c_str());
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(executor, node, "move_group");
  while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    RCLCPP_INFO_ONCE(node->get_logger(), "service not available, waiting again...");
  }
  // TODO(speralta):(Use MoveitConfigBuilder to set params in launch file).
  RCLCPP_INFO(node->get_logger(), "Setting parameters");
  auto list_result =
      parameters_client->list_parameters({ "move_group", "robot_description_planning", "interbotix_gripper",
                                           "interbotix_arm", "", "moveit_simple_controller_managers" },
                                         0);
  auto all_params = parameters_client->get_parameters(list_result.names);
  node->set_parameters(all_params);

  auto parameters = parameters_client->get_parameters({"robot_description_semantic"});
  node->set_parameters(parameters);
  // Wait for ApplyPlanningScene service
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Add table and object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;

  if (node->get_parameter_or("spawn_table", false))
  {
    spawnObject(psi, createTable(node));
  }
  std::vector<std::string> spawn_objs;
  node->get_parameter("spawn_objs", spawn_objs);


  for (std::string obj : spawn_objs)
  {


    moveit_msgs::msg::CollisionObject cobj = createObject(node, obj);
    RCLCPP_WARN(node->get_logger(), " COBJ %s RESULT: (%.2f, %.2f, %.2f)", cobj.header.frame_id.c_str(),
                cobj.primitive_poses.back().position.x, cobj.primitive_poses.back().position.y,
                cobj.primitive_poses.back().position.z);
    spawnObject(psi, cobj);
    // sleep for half a second
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // if (deep_pick_place_task.plan())
    // {
    //   RCLCPP_INFO(node->get_logger(), "Planning succeded");
    //   if (node->get_parameter_or("execute", false))
    //   {
    //     if (deep_pick_place_task.execute())
    //     {
    //       RCLCPP_INFO(node->get_logger(), "Execution complete");
    //     }
    //     else
    //     {
    //       RCLCPP_INFO(node->get_logger(), "Execution failed");
    //       break;
    //     }
    //   }
    //   else
    //   {
    //     RCLCPP_INFO(node->get_logger(), "Execution disabled");
    //   }
    // }
    // else
    // {
    //   RCLCPP_INFO(node->get_logger(), "Planning failed");
    //   break;
    // }
  }

  // Keep introspection alive

  executor->spin();
  rclcpp::shutdown();
  return 0;
}