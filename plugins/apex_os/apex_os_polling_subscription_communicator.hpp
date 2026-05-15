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

#ifndef PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "performance_test/experiment_configuration/qos_abstraction.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"

namespace performance_test
{

/// Communication plugin for Apex.OS using waitsets for the subscription side.
template<class Msg>
class ApexOSPollingSubscriptionSubscriber : public Subscriber
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename Msg::RosType;

  explicit ApexOSPollingSubscriptionSubscriber(const ExperimentConfiguration & ec)
  : m_ec(ec),
    m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get()),
    m_polling_subscription(m_node->create_polling_subscription<DataType>(
        ec.topic_name + ec.sub_topic_postfix(),
        m_ROS2QOSAdapter)),
    m_waitset(std::make_unique<rclcpp::Waitset<>>(m_polling_subscription))
  {
  }

  void prepare() override
  {
    if (m_ec.expected_num_pubs > 0) {
      m_polling_subscription->wait_for_matched(
        m_ec.expected_num_pubs,
        m_ec.wait_for_matched_timeout,
        std::greater_equal<size_t>(),
        0U,
        std::greater_equal<size_t>(),
        std::chrono::milliseconds(10 * m_ec.number_of_subscribers));
    }
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    const auto wait_ret = m_waitset->wait(std::chrono::milliseconds(100), false);
    if (wait_ret.any()) {
      take(listener);
    }
  }

  void take(MessageReceivedListener & listener) override
  {
    const auto loaned_msg = m_polling_subscription->take(RCLCPP_LENGTH_UNLIMITED);
    const auto received_time = now_int64_t();
    for (const auto msg : loaned_msg) {
      if (msg.info().valid()) {
        listener.on_message_received(
          msg.data().time,
          received_time,
          msg.data().id,
          sizeof(DataType)
        );
      }
    }
  }

private:
  const ExperimentConfiguration & m_ec;
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  using PollingSubscriptionType = ::rclcpp::PollingSubscription<DataType>;
  std::shared_ptr<PollingSubscriptionType> m_polling_subscription;
  std::unique_ptr<rclcpp::Waitset<>> m_waitset;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_
