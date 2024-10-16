// Copyright 2024 Sebastian Peralta
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros2_agents/actor.hpp>
#include <ros2_agents/observer.hpp>

namespace mbodied
{

/**
 * @brief This class is responsible for recieving observations and generatoring
 * actions.
 *
 * @tparam ObsT
 * @tparam ActionT
 * @tparam SrcTs
 */
template <typename ObsT, typename ActionT, typename... SrcTs>
class Agent : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Agent object
   *
   * @param options
   * @param src_topics Source topics to subscribe to and generate observations from.
   */
  Agent(rclcpp::NodeOptions& options)
    : Node("agent", options)

  {
    this->declare_parameter("world_frame", "world");
    for (int i = 0; i < static_cast<int>(sizeof...(SrcTs)); i++)
    {
      this->declare_parameter("src_frame" + std::to_string(i), "camera_locobot_link");
    }
    this->declare_parameter("instruction", "pick block");
    
    const auto obs_from_srcs_callback = [this](std::shared_ptr<SrcTs>... src_msgs) { return obsFromSrcs(src_msgs...); };
    observer_ = std::make_shared<Observer<ObsT, SrcTs...>>(options, obs_from_srcs_callback);
    actor_ = std::make_shared<Actor<ActionT>>(options, std::bind(&Agent::actionFromObs, this, observer_));
  }

  virtual void run()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(actor_);
    exec.add_node(observer_);
    exec.spin();
  }

  /**
   * @brief Function that generates an action from an observer.
   *
   * @param observer  Observer that can be called multiple times to generate observations.
   * @return ActionT::Feedback::SharedPtr
   */
  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  /**
   * @brief Function that generates an observation from a set of source messages.
   *
   * @param srcs Source messages to generate observation from.
   * @return std::unique_ptr<ObsT>
   */
  virtual std::unique_ptr<ObsT> obsFromSrcs(std::shared_ptr<SrcTs>... srcs) = 0;

protected:


  std::string get_instruction() {
    return this->get_parameter("instruction").as_string();
  }
  
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;
};
}  // namespace mbodied
