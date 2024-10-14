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


/**
 * @brief Helper functions for creating generic subscriptions.
 * 
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include <memory>
#include <any>

namespace mbodied
{
namespace util
{
/**
 * @brief Adds a subscription that stores message in a shared_ptr argument and sets
 *  recieved_observation to true.
 *
 * @tparam SrcT
 * @param node
 * @param topic
 * @param callback_group
 * @param src_msg Pointer to message to store subscription in.
 * @param received_observation Pointer to bool that is set to true when message is recieved.
 * @return std::shared_ptr<rclcpp::Subscription<SrcT>>
 */
template <typename SrcT>
std::shared_ptr<rclcpp::Subscription<SrcT>> generic_subscription(rclcpp::Node* node, const std::string& topic,
                                                                 rclcpp::CallbackGroup::SharedPtr callback_group,
                                                                 SrcT* src_msg_dest, bool* received_observation)
{
  auto callback = [node, src_msg_dest, received_observation](const std::shared_ptr<SrcT> msg) {
    *src_msg_dest = *msg;
    *received_observation = true;
  };
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  return node->create_subscription<SrcT>(topic, 10, callback, options);
}

/**
 * @brief Creates and returns tuple of subscriptions that subscribe to src_topics
 *  and store the messages in src_msgs. Additionally, sets recieved_first_srcs to true
 *
 * @tparam Tuple
 * @tparam Is compile-time sequence of integers.
 * @param node
 * @param src_topics
 * @param callback_group
 * @param recieved_first_srcs
 * @param src_msgs
 * @return auto
 */
template <typename Tuple, size_t... Is>
auto addSubscriptions(rclcpp::Node* node, const std::array<std::string, sizeof...(Is)>& src_topics,
                      rclcpp::CallbackGroup::SharedPtr callback_group, Tuple& src_msgs,
                      std::array<bool, sizeof...(Is)>& recieved_first_srcs, std::index_sequence<Is...>)
{
  return std::make_tuple(generic_subscription(node, std::get<Is>(src_topics), callback_group,
                                              std::get<Is>(src_msgs).get(), &std::get<Is>(recieved_first_srcs))...);
}
}  // namespace util
}  // namespace mbodied
