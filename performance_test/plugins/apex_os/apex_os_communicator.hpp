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

#include <chrono>
#include <memory>
#include <ratio>
#include <utility>
#include <functional>

#ifndef PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_COMMUNICATOR_HPP_

#include <apexcpp/node_state.hpp>
#include <cpputils/optional.hpp>
#include <executor/executable_item.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_common/ros2_qos_adapter.hpp"
#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/experiment_metrics/subscriber_stats.hpp"
#include "performance_test/utilities/message_initializer.hpp"

namespace performance_test
{
template<class MsgType>
class PublisherItem : public apex::executor::executable_item
{
public:
  PublisherItem(
    rclcpp::Node & node, apex::NodeState & node_state,
    PublisherStats & stats, const ExperimentConfiguration & ec)
  : apex::executor::executable_item(node, node_state),
    m_ec(ec),
    m_stats(stats),
    m_publisher(node.create_publisher<MsgType>(
        ec.topic_name + ec.pub_topic_postfix(),
        ROS2QOSAdapter(ec.qos).get())),
    m_message_initializer(ec)
  {}

  void prepare()
  {
    if (m_ec.expected_num_subs > 0) {
      m_publisher->wait_for_matched(
        m_ec.expected_num_subs,
        m_ec.wait_for_matched_timeout);
    }
  }

private:
  void execute_impl() override
  {
    if (m_ec.use_loaned_samples) {
      if (!m_publisher->can_loan_messages()) {
        throw std::runtime_error(
                "RMW implementation does not support zero copy!");
      }
      auto borrowed_message{m_publisher->borrow_loaned_message()};
      m_message_initializer.init_msg(
        *borrowed_message,
        m_timestamp_provider,
        m_stats.next_sample_id());
      m_publisher->publish(std::move(borrowed_message));
      m_stats.on_message_sent();
    } else {
      m_message_initializer.init_msg(
        m_data,
        m_timestamp_provider,
        m_stats.next_sample_id());
      m_publisher->publish(m_data);
      m_stats.on_message_sent();
    }
  }

  MsgType m_data;
  const ExperimentConfiguration & m_ec;
  PublisherStats & m_stats;
  const typename rclcpp::Publisher<MsgType>::SharedPtr m_publisher;
  MessageInitializer m_message_initializer;
  PublisherTimestampProvider m_timestamp_provider;
};

template<class MsgType>
class SubscriberItem : public apex::executor::executable_item
{
public:
  SubscriberItem(
    rclcpp::Node & node, apex::NodeState & node_state,
    SubscriberStats & stats, const ExperimentConfiguration & ec)
  : apex::executor::executable_item(node, node_state),
    m_ec(ec),
    m_stats(stats),
    m_subscription(node.template create_polling_subscription<MsgType>(
        ec.topic_name + ec.sub_topic_postfix(),
        ROS2QOSAdapter(ec.qos).get()))
  {
    if (m_ec.roundtrip_mode == RoundTripMode::RELAY) {
      m_publisher.emplace(
        node.create_publisher<MsgType>(
          m_ec.topic_name + m_ec.pub_topic_postfix(),
          ROS2QOSAdapter(m_ec.qos).get()));
    }
  }

  void prepare()
  {
    if (this->m_ec.expected_num_pubs > 0) {
      m_subscription->wait_for_matched(
        this->m_ec.expected_num_pubs,
        this->m_ec.wait_for_matched_timeout,
        std::greater_equal<size_t>(), 0U, std::greater_equal<size_t>(),
        std::chrono::milliseconds(10 * this->m_ec.number_of_subscribers));
    }
  }

  apex::executor::subscription_list
  get_triggering_subscriptions_impl() const override
  {
    return {m_subscription};
  }

private:
  void execute_impl() override
  {
    const auto loaned_msg = m_subscription->take();
    const auto received_time = now_int64_t();
    for (const auto msg : loaned_msg) {
      if (msg.info().valid()) {
        callback(msg.data(), received_time);
      }
    }
  }

  template<class T>
  void callback(const T & data, const std::int64_t received_time)
  {
    static_assert(
      std::is_same<typename MsgType::BorrowedType,
      typename std::remove_cv<
        typename std::remove_reference<T>::type>::type>::value,
      "Parameter type passed to callback() does not match");
    if (m_ec.roundtrip_mode == RoundTripMode::RELAY) {
      m_publisher.value()->publish(data);
    } else {
      m_stats.on_message_received(
        data.time, received_time, data.id, sizeof(MsgType));
    }
  }

  const ExperimentConfiguration & m_ec;
  SubscriberStats & m_stats;
  const typename rclcpp::PollingSubscription<MsgType>::SharedPtr m_subscription;
  apex::optional<typename rclcpp::Publisher<MsgType>::SharedPtr> m_publisher;
};

class ApexOsPublisherEntity
{
public:
  virtual std::shared_ptr<apex::executor::executable_item>
  get_publisher_item()
  {
    return nullptr;
  }
  virtual void prepare() {}
};

class ApexOsSubscriberEntity
{
public:
  virtual std::shared_ptr<apex::executor::executable_item>
  get_subscriber_item()
  {
    return nullptr;
  }
  virtual void prepare() {}
};

template<typename MsgType>
class ApexOsPublisher : public ApexOsPublisherEntity
{
public:
  ApexOsPublisher(PublisherStats & stats, const ExperimentConfiguration & ec)
  : m_node("apex_os_publisher_node"),
    m_node_state(&m_node, std::chrono::seconds::max()),
    m_publisher_item(std::make_shared<PublisherItem<MsgType>>(
        m_node, m_node_state, stats, ec)) {}

  std::shared_ptr<apex::executor::executable_item>
  get_publisher_item() override
  {
    return m_publisher_item;
  }

  void prepare() override
  {
    m_publisher_item->prepare();
  }

private:
  rclcpp::Node m_node;
  apex::NodeState m_node_state;
  std::shared_ptr<PublisherItem<MsgType>> m_publisher_item;
};

template<typename MsgType>
class ApexOsSubscriber : public ApexOsSubscriberEntity
{
public:
  ApexOsSubscriber(SubscriberStats & stats, const ExperimentConfiguration & ec)
  : m_node("apex_os_subscriber_node"),
    m_node_state(&m_node, std::chrono::seconds::max()),
    m_subscriber_item(std::make_shared<SubscriberItem<MsgType>>(
        m_node, m_node_state, stats, ec)) {}

  std::shared_ptr<apex::executor::executable_item>
  get_subscriber_item() override
  {
    return m_subscriber_item;
  }

  void prepare() override
  {
    m_subscriber_item->prepare();
  }

private:
  rclcpp::Node m_node;
  apex::NodeState m_node_state;
  std::shared_ptr<SubscriberItem<MsgType>> m_subscriber_item;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__APEX_OS__APEX_OS_COMMUNICATOR_HPP_
