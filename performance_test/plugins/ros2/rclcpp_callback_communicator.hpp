// Copyright 2017-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"

namespace performance_test
{

template<class Msg, class Executor>
class RclcppCallbackSubscriber : public Subscriber
{
public:
  using DataType = typename Msg::RosType;

  explicit RclcppCallbackSubscriber(const ExperimentConfiguration & ec)
  : m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get()),
    m_subscription(m_node->create_subscription<DataType>(
        ec.topic_name + ec.sub_topic_postfix(),
        m_ROS2QOSAdapter,
        [this](const typename DataType::SharedPtr data) {this->callback(data);}))
  {
    m_executor.add_node(this->m_node);
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (ec.expected_num_pubs > 0) {
      m_subscription->wait_for_matched(
        ec.expected_num_pubs,
        ec.wait_for_matched_timeout);
    }
#endif
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    m_listener = &listener;
    m_executor.spin_once(std::chrono::milliseconds(100));
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  Executor m_executor;
  std::shared_ptr<::rclcpp::Subscription<DataType>> m_subscription;
  MessageReceivedListener * m_listener;

  void callback(const typename DataType::SharedPtr data)
  {
    callback(*data);
  }

  template<class T>
  void callback(const T & data)
  {
    const auto received_time = now_int64_t();
    static_assert(
      std::is_same<DataType,
      typename std::remove_cv<
        typename std::remove_reference<T>::type>::type>::value,
      "Parameter type passed to callback() does not match");
    m_listener->on_message_received(
      data.time,
      received_time,
      data.id,
      sizeof(DataType)
    );
  }
};

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
template<class Msg>
using RclcppSingleThreadedExecutorSubscriber =
  RclcppCallbackSubscriber<Msg, rclcpp::executors::SingleThreadedExecutor>;
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
template<class Msg>
using RclcppStaticSingleThreadedExecutorSubscriber =
  RclcppCallbackSubscriber<Msg, rclcpp::executors::StaticSingleThreadedExecutor>;
#endif
}  // namespace performance_test
#endif  // PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
