// Copyright 2022-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_
#define PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

#include "performance_test/experiment_configuration/qos_abstraction.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "performance_test/utilities/msg_traits.hpp"
#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"

namespace performance_test
{
template<class Msg>
class RclcppPublisher : public Publisher
{
public:
  using DataType = typename Msg::RosType;

  explicit RclcppPublisher(const ExperimentConfiguration & ec)
  : Publisher(ec),
    m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get()),
    m_publisher(m_node->create_publisher<DataType>(
        ec.topic_name + ec.pub_topic_postfix(),
        m_ROS2QOSAdapter))
  {
  }

  void prepare() override
  {
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (m_ec.expected_num_subs > 0) {
      m_publisher->wait_for_matched(
        m_ec.expected_num_subs,
        m_ec.wait_for_matched_timeout);
    }
#endif
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    init_msg(m_data, timestamp_provider, sample_id);
    m_publisher->publish(m_data);
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    #ifdef PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED
    if (!m_publisher->can_loan_messages()) {
      throw std::runtime_error("RMW implementation does not support zero copy!");
    }
    auto borrowed_message{m_publisher->borrow_loaned_message()};
    init_msg(borrowed_message.get(), timestamp_provider, sample_id);
    m_publisher->publish(std::move(borrowed_message));
    #else
    throw std::runtime_error("ROS2 distribution does not support zero copy!");
    #endif
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  std::shared_ptr<::rclcpp::Publisher<DataType>> m_publisher;
  DataType m_data;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_
