// Copyright 2021-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__ICEORYX__ICEORYX_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__ICEORYX__ICEORYX_COMMUNICATOR_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <vector>

#include <iceoryx_posh/popo/untyped_publisher.hpp>
#include <iceoryx_posh/popo/untyped_subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iceoryx_dust/cxx/std_string_support.hpp>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{
class IceoryxCommunicator
{
public:
  explicit IceoryxCommunicator(const ExperimentConfiguration & ec)
  {
    static bool initialized = false;
    static std::mutex m;
    std::lock_guard lock(m);
    if (!initialized) {
      initialized = true;
      if (ec.number_of_subscribers == 0) {
        iox::runtime::PoshRuntime::initRuntime("iox-perf-test-pub");
      } else if (ec.number_of_publishers == 0) {
        iox::runtime::PoshRuntime::initRuntime("iox-perf-test-sub");
      } else {
        iox::runtime::PoshRuntime::initRuntime("iox-perf-test-intra");
      }
    }
  }
};

template<class Msg>
class IceoryxPublisher : public IceoryxCommunicator, public Publisher
{
public:
  using DataType = typename Msg::RosType;

  explicit IceoryxPublisher(const ExperimentConfiguration & ec)
  : IceoryxCommunicator(ec),
    Publisher(ec),
    m_publisher(
      iox::capro::ServiceDescription{
      iox::into<iox::lossy<iox::capro::IdString_t>>(Msg::msg_name()),
      iox::into<iox::lossy<iox::capro::IdString_t>>(ec.topic_name),
      "Object"
    },
      iox::popo::PublisherOptions{}) {}

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    publish_loaned(timestamp_provider, sample_id);
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    m_publisher.loan(sizeof(DataType))
    .and_then(
      [&, this](auto & userPayload) {
        auto sample = static_cast<DataType *>(userPayload);
        this->init_msg(*sample, timestamp_provider, sample_id);
        m_publisher.publish(userPayload);
      })
    .or_else(
      [](auto &) {
        throw std::runtime_error("Failed to write to sample");
      });
  }

private:
  iox::popo::UntypedPublisher m_publisher;
  DataType m_data;
};

template<class Msg>
class IceoryxSubscriber : public IceoryxCommunicator, public Subscriber
{
public:
  using DataType = typename Msg::RosType;

  explicit IceoryxSubscriber(const ExperimentConfiguration & ec)
  : IceoryxCommunicator(ec),
    m_subscriber(
      iox::capro::ServiceDescription{
      iox::into<iox::lossy<iox::capro::IdString_t>>(Msg::msg_name()),
      iox::into<iox::lossy<iox::capro::IdString_t>>(ec.topic_name),
      "Object"
    },
      subscriber_options(ec))
  {
    m_waitset.attachEvent(m_subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED)
    .or_else(
      [](
        auto) {
        std::cerr << "unable to attach Event DATA_RECEIVED to iceoryx Waitset" << std::endl;
        std::exit(EXIT_FAILURE);
      });
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    if (m_subscriber.getSubscriptionState() == iox::SubscribeState::SUBSCRIBED) {
      auto eventVector = m_waitset.timedWait(iox::units::Duration::fromSeconds(15));
      for (auto & event : eventVector) {
        if (event->doesOriginateFrom(&m_subscriber)) {
          while (m_subscriber.hasData()) {
            this->take(listener);
          }
        }
      }
    } else {
      std::cout << "Not subscribed!" << std::endl;
    }
  }

  void take(MessageReceivedListener & listener) override
  {
    m_subscriber.take()
    .and_then(
      [&, this](const void * data) {
        const auto received_time = now_int64_t();
        auto receivedSample = static_cast<const DataType *>(data);
        listener.on_message_received(
          receivedSample->time,
          received_time,
          receivedSample->id,
          sizeof(DataType)
        );
        m_subscriber.release(data);
      })
    .or_else(
      [](auto & result) {
        std::cerr << result << std::endl;
        if (result !=
        iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE)
        {
          throw std::runtime_error(
            "Error: received Chunk not available");
        }
      });
  }

private:
  static iox::popo::SubscriberOptions subscriber_options(const ExperimentConfiguration & ec)
  {
    iox::popo::SubscriberOptions options;
    options.queueCapacity = ec.qos.history_depth;
    return options;
  }

  iox::popo::UntypedSubscriber m_subscriber;
  iox::popo::WaitSet<> m_waitset;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__ICEORYX__ICEORYX_COMMUNICATOR_HPP_
