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

#ifndef PERFORMANCE_TEST__PLUGINS__CYCLONEDDS_CXX__CYCLONEDDS_CXX_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__CYCLONEDDS_CXX__CYCLONEDDS_CXX_COMMUNICATOR_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <dds/dds.hpp>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{

/**
 * \brief Apply the QOSAbstraction to the data reader or writer QOS.
 *
 * \param rw_qos The data reader or writer QOS.
 * \param ec The experiment configuration.
 */
template<class CycloneDDSCXXQos>
void apply_cylonedds_cxx_qos(
  CycloneDDSCXXQos & rw_qos,
  const ExperimentConfiguration & ec
)
{
  const QOSAbstraction qos = ec.qos;

  if (qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
    rw_qos.policy(dds::core::policy::Reliability::BestEffort());
  } else if (qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
    rw_qos.policy(dds::core::policy::Reliability::Reliable());
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }

  if (qos.durability == QOSAbstraction::Durability::VOLATILE) {
    rw_qos.policy(dds::core::policy::Durability::Volatile());
  } else if (qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
    rw_qos.policy(dds::core::policy::Durability::TransientLocal());
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }

  if (qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
    rw_qos.policy(dds::core::policy::History::KeepAll());
  } else if (qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
    rw_qos.policy(dds::core::policy::History::KeepLast(qos.history_depth));
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }
}

class CycloneDDSCXXResourceManager
{
public:
  static CycloneDDSCXXResourceManager & get()
  {
    static CycloneDDSCXXResourceManager instance;
    return instance;
  }

  CycloneDDSCXXResourceManager(CycloneDDSCXXResourceManager const &) = delete;
  CycloneDDSCXXResourceManager(CycloneDDSCXXResourceManager &&) = delete;
  CycloneDDSCXXResourceManager & operator=(CycloneDDSCXXResourceManager const &) = delete;
  CycloneDDSCXXResourceManager & operator=(CycloneDDSCXXResourceManager &&) = delete;

  dds::domain::DomainParticipant cyclonedds_cxx_participant(
    const ExperimentConfiguration & ec) const
  {
    std::lock_guard<std::mutex> lock(m_global_mutex);

    // CycloneDDS-CXX has its own reference-counting mechanism
    if (m_cyclonedds_cxx_participant.is_nil()) {
      ddsi_config config;
      if (ec.use_shared_memory) {
        config.enable_shm = true;
      }
      m_cyclonedds_cxx_participant = dds::domain::DomainParticipant(
        ec.dds_domain_id,
        dds::domain::qos::DomainParticipantQos(),
        nullptr,
        dds::core::status::StatusMask::none(),
        config
      );
    }
    return m_cyclonedds_cxx_participant;
  }

private:
  CycloneDDSCXXResourceManager() {}

  mutable dds::domain::DomainParticipant m_cyclonedds_cxx_participant{dds::core::null};
  mutable std::mutex m_global_mutex;
};

template<class Msg>
class CycloneDDSCXXPublisher : public Publisher
{
public:
  using DataType = typename Msg::CycloneDDSCXXType;

  explicit CycloneDDSCXXPublisher(const ExperimentConfiguration & ec)
  : Publisher(ec),
    m_participant(CycloneDDSCXXResourceManager::get().cyclonedds_cxx_participant(ec)),
    m_publisher(m_participant),
    m_datawriter(make_cyclonedds_cxx_datawriter<DataType>(
        m_participant, m_publisher, ec))
  {
    if (ec.use_loaned_samples && !m_datawriter.delegate()->is_loan_supported()) {
      throw std::runtime_error("Zero-copy transfer is not supported.");
    }
  }

  ~CycloneDDSCXXPublisher()
  {
    this->m_datawriter = dds::core::null;
    this->m_publisher = dds::core::null;
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    DataType sample;
    init_msg(sample, timestamp_provider, sample_id);
    m_datawriter->write(sample);
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    DataType & loaned_sample = m_datawriter.delegate()->loan_sample();
    init_msg(loaned_sample, timestamp_provider, sample_id);
    m_datawriter->write(loaned_sample);
  }

private:
  dds::domain::DomainParticipant m_participant;
  dds::pub::Publisher m_publisher;
  dds::pub::DataWriter<DataType> m_datawriter;

  template<typename DataType>
  static dds::pub::DataWriter<DataType> make_cyclonedds_cxx_datawriter(
    const dds::domain::DomainParticipant & participant,
    const dds::pub::Publisher & publisher,
    const ExperimentConfiguration & ec
  )
  {
    std::string topic_name = ec.topic_name + ec.pub_topic_postfix();
    auto topic = dds::topic::Topic<DataType>(participant, topic_name);

    dds::pub::qos::DataWriterQos dw_qos = publisher.default_datawriter_qos();
    apply_cylonedds_cxx_qos(dw_qos, ec);

    return dds::pub::DataWriter<DataType>(publisher, topic, dw_qos);
  }
};

template<class Msg>
class CycloneDDSCXXSubscriber : public Subscriber
{
public:
  using DataType = typename Msg::CycloneDDSCXXType;

  explicit CycloneDDSCXXSubscriber(const ExperimentConfiguration & ec)
  : m_participant(CycloneDDSCXXResourceManager::get().cyclonedds_cxx_participant(ec)),
    m_subscriber(m_participant),
    m_datareader(make_cyclonedds_cxx_datareader<DataType>(
        m_participant, m_subscriber, ec)),
    m_read_condition(m_datareader,
      dds::sub::status::SampleState::not_read()),
    m_waitset()
  {
    m_waitset.attach_condition(m_read_condition);
  }

  ~CycloneDDSCXXSubscriber()
  {
    this->m_datareader = dds::core::null;
    this->m_subscriber = dds::core::null;
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    // Wait for the data to become available. This is the only condition, so no need to inspect the
    // returned list of triggered conditions.
    try {
      m_waitset.wait(dds::core::Duration(15, 0));
      this->take(listener);
    } catch (dds::core::TimeoutError &) {
    }
  }

  void take(MessageReceivedListener & listener) override
  {
    dds::sub::LoanedSamples<DataType> samples = m_datareader->take();
    const auto received_time = now_int64_t();
    for (auto & sample : samples) {
      if (sample->info().valid()) {
        listener.on_message_received(
          sample->data().time(),
          received_time,
          sample->data().id(),
          sizeof(DataType)
        );
      }
    }
  }

private:
  dds::domain::DomainParticipant m_participant;
  dds::sub::Subscriber m_subscriber;
  dds::sub::DataReader<DataType> m_datareader;
  dds::sub::cond::ReadCondition m_read_condition;
  dds::core::cond::WaitSet m_waitset;

  template<typename DataType>
  static dds::sub::DataReader<DataType> make_cyclonedds_cxx_datareader(
    const dds::domain::DomainParticipant & participant,
    const dds::sub::Subscriber & subscriber,
    const ExperimentConfiguration & ec
  )
  {
    std::string topic_name = ec.topic_name + ec.sub_topic_postfix();
    auto topic = dds::topic::Topic<DataType>(participant, topic_name);

    dds::sub::qos::DataReaderQos dr_qos = subscriber.default_datareader_qos();
    apply_cylonedds_cxx_qos(dr_qos, ec);

    return dds::sub::DataReader<DataType>(subscriber, topic, dr_qos);
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__CYCLONEDDS_CXX__CYCLONEDDS_CXX_COMMUNICATOR_HPP_
