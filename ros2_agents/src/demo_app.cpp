#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ros2_agents/client.hpp>
#include <ros2_agents_interfaces/action/mbodied.hpp>
#include <moveit/planning_scene/planning_scene.h>
namespace chrono = std::chrono;

using ros2_agents_interfaces::action::Mbodied;

// C++ 20 convenience for legacy CXX compilers.
#define TIME_DIFF(tic,                                                                    \
                  toc) static_cast<int>(chrono::duration_cast<chrono::milliseconds>(toc - \
                                                                                    tic).count())
#define CLAMP(x, xmin, xmax) std::max(xmin, std::min(xmax, x))


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto agent_client_node = std::make_shared<mbodied::AgentClient<Mbodied> >(options);

  // Spin in the background for planning scene to work.
  rclcpp::executors::MultiThreadedExecutor executor;

  /* *INDENT-OFF* */
  executor.add_node(agent_client_node);
  std::thread([&executor]() {executor.spin();}).detach();
 /* *INDENT-ON* */
  agent_client_node->init();

  for (int i = 0; i < agent_client_node->get_param<int>("num_iterations"); ++i)
  {
    if (!rclcpp::ok())
    {
      break;
    }
    RCLCPP_INFO(agent_client_node->get_logger(), "*** Iteration %i", i);
    const auto tic = chrono::steady_clock::now();
    agent_client_node->act();
    const auto toc = chrono::steady_clock::now();
    RCLCPP_INFO(agent_client_node->get_logger(), "Action took %i ms", TIME_DIFF(tic, toc));
    std::this_thread::sleep_for(chrono::milliseconds(agent_client_node->get_param<int>("sleep_ms")));
  }
  rclcpp::shutdown();
  return 0;
}
