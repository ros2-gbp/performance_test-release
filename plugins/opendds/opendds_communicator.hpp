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

#ifndef PERFORMANCE_TEST__PLUGINS__OPENDDS__OPENDDS_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__OPENDDS__OPENDDS_COMMUNICATOR_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <dds/DCPS/RTPS/RtpsDiscovery.h>
#include <dds/DCPS/transport/framework/TransportRegistry.h>
#include <dds/DCPS/transport/rtps_udp/RtpsUdpInst_rch.h>
#include <dds/DCPS/transport/rtps_udp/RtpsUdpInst.h>
#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsPublicationC.h>
#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>
#include <dds/DCPS/StaticIncludes.h>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

#ifdef LENGTH_UNLIMITED
#undef LENGTH_UNLIMITED
#endif

#define BLOCK_SEC 10
#define BLOCK_NANO_SEC 0

namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for OpenDDS data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a partially specified QOS from the topic
 * or similar higher level entity and just changes some settings from these.
 */

class OpenDdsQOSAdapter
{
public:
  explicit OpenDdsQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  void apply_dr(DDS::DataReaderQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

  void apply_dw(DDS::DataWriterQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

private:
  const QOSAbstraction m_qos;
};

class OpenDDSResourceManager
{
public:
  static OpenDDSResourceManager & get()
  {
    static OpenDDSResourceManager instance;
    return instance;
  }

  OpenDDSResourceManager(OpenDDSResourceManager const &) = delete;
  OpenDDSResourceManager(OpenDDSResourceManager &&) = delete;
  OpenDDSResourceManager & operator=(OpenDDSResourceManager const &) = delete;
  OpenDDSResourceManager & operator=(OpenDDSResourceManager &&) = delete;

  DDS::DomainParticipant_ptr opendds_participant(const ExperimentConfiguration & ec) const
  {
    std::lock_guard<std::mutex> lock(m_global_mutex);

    if (CORBA::is_nil(m_opendds_participant)) {
      DDS::DomainParticipantFactory_var dpf = TheParticipantFactory;

      OpenDDS::DCPS::TransportConfig_rch config =
        OpenDDS::DCPS::TransportRegistry::instance()->create_config("ApexAiConfig");
      OpenDDS::DCPS::TransportInst_rch inst =
        OpenDDS::DCPS::TransportRegistry::instance()->create_inst("rtps_tran", "rtps_udp");
      OpenDDS::DCPS::RtpsUdpInst_rch rui =
        OpenDDS::DCPS::static_rchandle_cast<OpenDDS::DCPS::RtpsUdpInst>(inst);
      rui->handshake_timeout_ = 1;

      config->instances_.push_back(inst);
      OpenDDS::DCPS::TransportRegistry::instance()->global_config(config);

      int domain = ec.dds_domain_id;
      bool multicast = true;
      unsigned int resend = 1;
      std::string partition, governance, permissions;
      int defaultSize = 0;

      OpenDDS::RTPS::RtpsDiscovery_rch disc;
      disc = OpenDDS::DCPS::make_rch<OpenDDS::RTPS::RtpsDiscovery>("RtpsDiscovery");
      rui->use_multicast_ = true;
      rui->local_address("127.0.0.1:");
      rui->multicast_interface_ = "lo";
      disc->sedp_multicast(true);

      TheServiceParticipant->add_discovery(
        OpenDDS::DCPS::static_rchandle_cast<OpenDDS::DCPS::Discovery>(disc));
      TheServiceParticipant->set_repo_domain(domain, disc->key());
      DDS::DomainParticipantQos dp_qos;
      dpf->get_default_participant_qos(dp_qos);
      m_opendds_participant = dpf->create_participant(
        ec.dds_domain_id,
        PARTICIPANT_QOS_DEFAULT,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);
    }
    return m_opendds_participant;
  }

  /**
   * \brief Creates a new OpenDDS publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void opendds_publisher(
    const ExperimentConfiguration & ec,
    DDS::Publisher_ptr & publisher,
    DDS::DataWriterQos & dw_qos) const
  {
    DDS::DomainParticipant_ptr participant = opendds_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);

    publisher = participant->create_publisher(
      PUBLISHER_QOS_DEFAULT,
      nullptr,
      OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (CORBA::is_nil(publisher)) {
      throw std::runtime_error("Failed to create publisher");
    }

    DDS::ReturnCode_t ret;
    ret = publisher->get_default_datawriter_qos(dw_qos);
    if (ret != DDS::RETCODE_OK) {
      throw std::runtime_error("Failed to get default datawriter qos");
    }
  }

  /**
   * \brief Creates a new OpenDDS subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void opendds_subscriber(
    const ExperimentConfiguration & ec,
    DDS::Subscriber_ptr & subscriber,
    DDS::DataReaderQos & dr_qos) const
  {
    DDS::DomainParticipant_ptr participant = opendds_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);

    subscriber = participant->create_subscriber(
      SUBSCRIBER_QOS_DEFAULT,
      nullptr,
      OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (CORBA::is_nil(subscriber)) {
      throw std::runtime_error("Failed to create subscriber");
    }

    DDS::ReturnCode_t ret;
    ret = subscriber->get_default_datareader_qos(dr_qos);
    if (ret != DDS::RETCODE_OK) {
      throw std::runtime_error("Failed to get default datareader qos");
    }
  }

private:
  OpenDDSResourceManager()
  : m_opendds_participant(nullptr) {}

  mutable std::mutex m_global_mutex;
  mutable DDS::DomainParticipant_ptr m_opendds_participant;
};

class OpenDDSResources {
public:
  template<class Topic>
  static DDS::Topic_ptr find_or_create_topic(
    const ExperimentConfiguration & ec,
    DDS::DomainParticipant_ptr participant)
  {
    DDS::Duration_t timeout;
    timeout.sec = 0;
    timeout.nanosec = 500;

    DDS::Topic_ptr topic = participant->find_topic(
      ec.topic_name.c_str(),
      timeout);

    if (CORBA::is_nil(topic)) {
      DDS::ReturnCode_t retcode;
      retcode = Topic::get_type_support()->register_type(
        participant,
        Topic::msg_name().c_str());
      if (retcode != DDS::RETCODE_OK) {
        throw std::runtime_error("failed to register type");
      }
      topic = participant->create_topic(
        ec.topic_name.c_str(),
        Topic::msg_name().c_str(),
        TOPIC_QOS_DEFAULT,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);
      if (CORBA::is_nil(topic)) {
        throw std::runtime_error("topic == nullptr");
      }
    }

    return topic;
  }
};

template<class Topic>
class OpenDDSPublisher : public Publisher
{
public:
  using DataType = typename Topic::OpenDDSTopicType;
  using DataWriterType = typename Topic::OpenDDSDataWriterType;

  explicit OpenDDSPublisher(const ExperimentConfiguration & ec)
  : m_participant(OpenDDSResourceManager::get().opendds_participant(ec)),
    m_datawriter(make_opendds_datawriter(ec, m_participant))
  {
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    init_msg(m_data, timestamp_provider, sample_id);
    auto retcode = m_datawriter->write(m_data, DDS::HANDLE_NIL);
    if (retcode != DDS::RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    throw std::runtime_error("This plugin does not support zero copy transfer");
  }

private:
  DDS::DomainParticipant_ptr m_participant;
  DataWriterType * m_datawriter;
  DataType m_data;

  DataWriterType * make_opendds_datawriter(
    const ExperimentConfiguration & ec,
    DDS::DomainParticipant_ptr participant)
  {
    DDS::Publisher_ptr publisher;
    DDS::DataWriterQos dw_qos;
    OpenDDSResourceManager::get().opendds_publisher(ec, publisher, dw_qos);

    OpenDdsQOSAdapter qos_adapter(ec.qos);
    qos_adapter.apply_dw(dw_qos);

    DDS::DataWriter_ptr datawriter = publisher->create_datawriter(
      OpenDDSResources::find_or_create_topic<Topic>(ec, participant),
      dw_qos,
      nullptr,
      OpenDDS::DCPS::DEFAULT_STATUS_MASK);
    if (CORBA::is_nil(datawriter)) {
      throw std::runtime_error("Could not create datawriter");
    }

    DataWriterType * typed_datawriter = DataWriterType::_narrow(datawriter);
    if (CORBA::is_nil(typed_datawriter)) {
      throw std::runtime_error("failed datawriter narrow");
    }
    return typed_datawriter;
  }
};

template<class Topic>
class OpenDDSSubscriber : public Subscriber
{
public:
  using DataType = typename Topic::OpenDDSTopicType;
  using DataReaderType = typename Topic::OpenDDSDataReaderType;
  using DataTypeSeq = typename Topic::OpenDDSDataTypeSeq;

  explicit OpenDDSSubscriber(const ExperimentConfiguration & ec)
  : m_participant(OpenDDSResourceManager::get().opendds_participant(ec)),
    m_datareader(make_opendds_datareader(ec, m_participant)),
    m_condition(m_datareader->get_statuscondition())
  {
    m_condition->set_enabled_statuses(DDS::DATA_AVAILABLE_STATUS);
    m_waitset.attach_condition(m_condition);
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    DDS::Duration_t wait_timeout = {15, 0};
    m_waitset.wait(m_condition_seq, wait_timeout);
    take(listener);
  }

  void take(MessageReceivedListener & listener) override
  {
    auto ret = m_datareader->take(
      m_data_seq, m_sample_info_seq, DDS::LENGTH_UNLIMITED,
      DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE,
      DDS::ANY_INSTANCE_STATE);
    const auto received_time = now_int64_t();
    if (ret == DDS::RETCODE_OK) {
      for (decltype(m_data_seq.length()) j = 0; j < m_data_seq.length(); ++j) {
        const auto & data = m_data_seq[j];
        if (m_sample_info_seq[j].valid_data) {
          listener.on_message_received(
            data.time,
            received_time,
            data.id,
            sizeof(DataType)
          );
        }
      }
      m_datareader->return_loan(
        m_data_seq,
        m_sample_info_seq);
    }
  }

private:
  DDS::DomainParticipant_ptr m_participant;
  DataReaderType * m_datareader;
  DDS::StatusCondition_ptr m_condition;
  DDS::WaitSet m_waitset;

  DDS::ConditionSeq m_condition_seq;
  DataTypeSeq m_data_seq;
  DDS::SampleInfoSeq m_sample_info_seq;

  DataReaderType * make_opendds_datareader(
    const ExperimentConfiguration & ec,
    DDS::DomainParticipant_ptr participant)
  {
    DDS::Subscriber_ptr subscriber;
    DDS::DataReaderQos dr_qos;
    OpenDDSResourceManager::get().opendds_subscriber(ec, subscriber, dr_qos);

    OpenDdsQOSAdapter qos_adapter(ec.qos);
    qos_adapter.apply_dr(dr_qos);

    DDS::DataReader_ptr datareader = subscriber->create_datareader(
      OpenDDSResources::find_or_create_topic<Topic>(ec, participant),
      dr_qos,
      nullptr,
      OpenDDS::DCPS::DEFAULT_STATUS_MASK);
    if (CORBA::is_nil(datareader)) {
      throw std::runtime_error("Could not create datareader");
    }

    DataReaderType * typed_datareader = DataReaderType::_narrow(datareader);
    if (typed_datareader == nullptr) {
      throw std::runtime_error("failed datareader narrow");
    }
    return typed_datareader;
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__OPENDDS__OPENDDS_COMMUNICATOR_HPP_
