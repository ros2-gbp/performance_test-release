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

#ifndef PERFORMANCE_TEST__PLUGINS__FASTDDS__FAST_RTPS_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__FASTDDS__FAST_RTPS_COMMUNICATOR_HPP_

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/core/LoanableSequence.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include <memory>
#include <mutex>
#include <vector>

#include "performance_test/experiment_configuration/qos_abstraction.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "performance_test/utilities/msg_traits.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for FastRTPS.
class FastRTPSQOSAdapter
{
public:
  explicit FastRTPSQOSAdapter(const QOSAbstraction qos, bool interprocess)
  : m_qos(qos),
    m_interprocess(interprocess)
  {}

  void apply(eprosima::fastdds::dds::DataWriterQos & wqos) const
  {
    apply_common(wqos);

    wqos.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
    wqos.reliable_writer_qos().times.heartbeatPeriod.fraction((200 * 1000 * 1000));
  }

  void apply(eprosima::fastdds::dds::DataReaderQos & rqos) const
  {
    apply_common(rqos);
  }

private:
  const QOSAbstraction m_qos;
  const bool m_interprocess;

  template<typename EntityQos>
  void apply_common(EntityQos & eqos) const
  {
    eqos.history().kind = history_kind();
    eqos.history().depth = history_depth();
    eqos.resource_limits().max_samples = resource_limits_samples();
    eqos.resource_limits().allocated_samples = resource_limits_samples();
    eqos.reliability().kind = reliability();
    eqos.durability().kind = durability();
    eqos.data_sharing().automatic();
    if (!m_interprocess) {eqos.data_sharing().off();}
  }

  inline eprosima::fastrtps::ReliabilityQosPolicyKind reliability() const
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

  inline eprosima::fastrtps::DurabilityQosPolicyKind durability() const
  {
    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

  inline eprosima::fastrtps::HistoryQosPolicyKind history_kind() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

  int32_t history_depth() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return static_cast<int32_t>(m_qos.history_depth);
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      // Keep all, keeps all. No depth required, but setting to dummy value.
      return 1;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

  int32_t resource_limits_samples() const
  {
    return static_cast<int32_t>(m_qos.history_depth);
  }
};

class FastDDSResourceManager
{
public:
  static FastDDSResourceManager & get()
  {
    static FastDDSResourceManager instance;
    return instance;
  }

  FastDDSResourceManager(FastDDSResourceManager const &) = delete;
  FastDDSResourceManager(FastDDSResourceManager &&) = delete;
  FastDDSResourceManager & operator=(FastDDSResourceManager const &) = delete;
  FastDDSResourceManager & operator=(FastDDSResourceManager &&) = delete;

  struct FastDDSGlobalResources
  {
    eprosima::fastdds::dds::DomainParticipant * participant;
    eprosima::fastdds::dds::Publisher * publisher;
    eprosima::fastdds::dds::Subscriber * subscriber;
    eprosima::fastdds::dds::Topic * topic;
  };

  const FastDDSGlobalResources & fastdds_resources(
    const ExperimentConfiguration & ec, eprosima::fastdds::dds::TypeSupport type) const
  {
    std::lock_guard<std::mutex> lock(m_global_mutex);

    if (!m_fastdds_resources.participant) {
      eprosima::fastdds::dds::DomainParticipantQos pqos;

      auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();

      // Load XML profiles and get default participant QoS
      factory->load_profiles();
      factory->get_default_participant_qos(pqos);

      // Participant is always considered alive
      pqos.wire_protocol().builtin.discovery_config.leaseDuration =
        eprosima::fastrtps::c_TimeInfinite;

      // Only tune transports if not tuned on the XML
      if (!(pqos.transport() == eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT.transport())) {
        // tuning system network stack
        pqos.transport().send_socket_buffer_size = 1048576;
        pqos.transport().listen_socket_buffer_size = 4194304;
        // set shm transport
        pqos.transport().use_builtin_transports = false;
        auto shm_transport =
          std::make_shared<eprosima::fastdds::rtps::SharedMemTransportDescriptor>();
        // changes to 6MB, default is 0.5MB, 512 * 1024 B
        shm_transport->segment_size(6 * 1024 * 1024);
        pqos.transport().user_transports.push_back(shm_transport);
        // set udp as fallback transport
        auto udp_transport =
          std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        pqos.transport().user_transports.push_back(udp_transport);
      }

      pqos.name("performance_test_fastDDS");

      m_fastdds_resources.participant = factory->create_participant(ec.dds_domain_id, pqos);
      if (m_fastdds_resources.participant == nullptr) {
        throw std::runtime_error("failed to create participant");
      }

      m_fastdds_resources.publisher = m_fastdds_resources.participant->create_publisher(
        eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
      if (m_fastdds_resources.publisher == nullptr) {
        throw std::runtime_error("failed to create publisher");
      }

      m_fastdds_resources.subscriber = m_fastdds_resources.participant->create_subscriber(
        eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
      if (m_fastdds_resources.subscriber == nullptr) {
        throw std::runtime_error("failed to create subscriber");
      }

      type.register_type(m_fastdds_resources.participant);

      auto topic_name = ec.topic_name + ec.pub_topic_postfix();
      m_fastdds_resources.topic = m_fastdds_resources.participant->create_topic(
        topic_name, type->getName(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
      if (m_fastdds_resources.topic == nullptr) {
        throw std::runtime_error("failed to create topic");
      }
    }
    return m_fastdds_resources;
  }

private:
  FastDDSResourceManager()
  : m_fastdds_resources{nullptr, nullptr, nullptr, nullptr} {}

  mutable std::mutex m_global_mutex;
  mutable FastDDSGlobalResources m_fastdds_resources;
};

template<class Topic>
class FastRTPSPublisher : public Publisher
{
public:
  using TopicType = typename Topic::EprosimaTopicType;
  using DataType = typename Topic::EprosimaType;

  explicit FastRTPSPublisher(const ExperimentConfiguration & ec)
  : Publisher(ec),
    m_resources(FastDDSResourceManager::get().fastdds_resources(
        ec, eprosima::fastdds::dds::TypeSupport(new TopicType()))),
    m_datawriter(create_datawriter(m_resources, ec))
  {
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    init_msg(m_data, timestamp_provider, sample_id);
    if (!m_datawriter->write(static_cast<void *>(&m_data))) {
      throw std::runtime_error("Failed to write sample");
    }
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    void * loaned_sample = nullptr;
    eprosima::fastrtps::types::ReturnCode_t ret = m_datawriter->loan_sample(loaned_sample);
    if (!ret) {
      throw std::runtime_error(
              "Failed to obtain a loaned sample, ERRORCODE is " + std::to_string(ret()));
    }
    DataType * sample = static_cast<DataType *>(loaned_sample);
    init_msg(*sample, timestamp_provider, sample_id);
    if (!m_datawriter->write(static_cast<void *>(sample))) {
      throw std::runtime_error("Failed to write sample");
    }
  }

private:
  FastDDSResourceManager::FastDDSGlobalResources m_resources;
  eprosima::fastdds::dds::DataWriter * m_datawriter;
  DataType m_data;

  static eprosima::fastdds::dds::DataWriter * create_datawriter(
    const FastDDSResourceManager::FastDDSGlobalResources & resources,
    const ExperimentConfiguration & ec
  )
  {
    const bool interprocess = ec.number_of_publishers == 0 || ec.number_of_subscribers == 0;
    const FastRTPSQOSAdapter qos(ec.qos, interprocess);

    eprosima::fastdds::dds::DataWriterQos wqos;
    resources.publisher->get_default_datawriter_qos(wqos);
    qos.apply(wqos);

    auto writer = resources.publisher->create_datawriter(resources.topic, wqos);
    if (!writer) {
      throw std::runtime_error("failed to create datawriter");
    }

    return writer;
  }
};

template<class Topic>
class FastRTPSSubscriber : public Subscriber
{
public:
  using TopicType = typename Topic::EprosimaTopicType;
  using DataType = typename Topic::EprosimaType;

  explicit FastRTPSSubscriber(const ExperimentConfiguration & ec)
  : m_resources(FastDDSResourceManager::get().fastdds_resources(
        ec, eprosima::fastdds::dds::TypeSupport(new TopicType()))),
    m_datareader(create_datareader(m_resources, ec))
  {
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    const eprosima::fastrtps::Duration_t secs_15{15, 0};
    m_datareader->wait_for_unread_message(secs_15);
    take(listener);
  }

  void take(MessageReceivedListener & listener) override
  {
    FASTDDS_SEQUENCE(DataSeq, DataType);
    DataSeq data_seq;
    eprosima::fastdds::dds::SampleInfoSeq info_seq;

    const auto ok_ret_code = eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK;
    while (ok_ret_code == m_datareader->take(data_seq, info_seq, 1)) {
      const auto received_time = now_int64_t();
      if (info_seq[0].valid_data) {
        listener.on_message_received(
          data_seq[0].time(),
          received_time,
          data_seq[0].id(),
          sizeof(DataType)
        );
      }
      m_datareader->return_loan(data_seq, info_seq);
    }
  }

private:
  FastDDSResourceManager::FastDDSGlobalResources m_resources;
  eprosima::fastdds::dds::DataReader * m_datareader;

  static eprosima::fastdds::dds::DataReader * create_datareader(
    const FastDDSResourceManager::FastDDSGlobalResources & resources,
    const ExperimentConfiguration & ec
  )
  {
    const bool interprocess = ec.number_of_publishers == 0 || ec.number_of_subscribers == 0;
    const FastRTPSQOSAdapter qos(ec.qos, interprocess);

    eprosima::fastdds::dds::DataReaderQos rqos;
    resources.subscriber->get_default_datareader_qos(rqos);
    qos.apply(rqos);

    auto reader = resources.subscriber->create_datareader(resources.topic, rqos);
    if (!reader) {
      throw std::runtime_error("failed to create datareader");
    }

    return reader;
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__FASTDDS__FAST_RTPS_COMMUNICATOR_HPP_
