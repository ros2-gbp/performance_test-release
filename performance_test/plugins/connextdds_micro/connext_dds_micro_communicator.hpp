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

#ifndef PERFORMANCE_TEST__PLUGINS__CONNEXTDDS_MICRO__CONNEXT_DDS_MICRO_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__CONNEXTDDS_MICRO__CONNEXT_DDS_MICRO_COMMUNICATOR_HPP_

#include <mutex>
#include <vector>

#include <rti_me_cpp.hxx>
#include <dds_cpp/dds_cpp_dpde.hxx>
#include <dds_cpp/dds_cpp_wh_sm.hxx>
#include <dds_cpp/dds_cpp_rh_sm.hxx>
#include <dds_cpp/dds_cpp_netio.hxx>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"


namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for Connext DDS Micro data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a partially specified QOS from the topic
 * or similar higher level entity and just changes some settings from these.
 */
class ConnextDDSMicroQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit ConnextDDSMicroQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}
  /**
   * \brief  Applies the abstract QOS to an existing QOS leaving unsupported values as they were.
   * \tparam ConnextDDSMicroQos The type of the QOS setting, for example data reader or data writer QOS.
   * \param qos The QOS settings to fill supported values in.
   */
  template<class ConnextDDSMicroQos>
  void apply(ConnextDDSMicroQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS_VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS_TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      // TODO(andreas.pasternak): Crash on keep all qos.
      // qos.history.kind = DDS_KEEP_ALL_HISTORY_QOS;
      qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
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

template<class Topic>
class RTIMicroDDSTopicManager {
public:
  static DDSTopic * register_topic(
    DDSDomainParticipant * m_participant,
    const ExperimentConfiguration & m_ec)
  {
    if (m_topic == nullptr) {
      auto retcode = Topic::ConnextDDSMicroType::TypeSupport::register_type(
        m_participant, Topic::msg_name().c_str());
      if (retcode != DDS_RETCODE_OK) {
        throw std::runtime_error("failed to register type");
      }
      m_topic = m_participant->create_topic(
        m_ec.topic_name.c_str(),
        Topic::msg_name().c_str(),
        DDS_TOPIC_QOS_DEFAULT,
        nullptr,
        DDS_STATUS_MASK_NONE);
      if (m_topic == nullptr) {
        throw std::runtime_error("topic == nullptr");
      }
      m_topic->enable();
    }
    return m_topic;
  }

private:
  static DDSTopic * m_topic;
};

template<class Topic>
DDSTopic * RTIMicroDDSTopicManager<Topic>::m_topic = nullptr;

class RTIMicroDDSResourceManager
{
public:
  static RTIMicroDDSResourceManager & get()
  {
    static RTIMicroDDSResourceManager instance;
    return instance;
  }

  RTIMicroDDSResourceManager(RTIMicroDDSResourceManager const &) = delete;
  RTIMicroDDSResourceManager(RTIMicroDDSResourceManager &&) = delete;
  RTIMicroDDSResourceManager & operator=(RTIMicroDDSResourceManager const &) = delete;
  RTIMicroDDSResourceManager & operator=(RTIMicroDDSResourceManager &&) = delete;

  DDSDomainParticipant * connext_DDS_micro_participant(
    const ExperimentConfiguration & ec) const
  {
    std::lock_guard<std::mutex> lock(m_global_mutex);

    if (!m_connext_dds_micro_participant) {
      DDSDomainParticipantFactory * factory = DDSDomainParticipantFactory::get_instance();
      RTRegistry * registry = factory->get_registry();

      registry->register_component("wh", WHSMHistoryFactory::get_interface(), nullptr, nullptr);
      registry->register_component("rh", RHSMHistoryFactory::get_interface(), nullptr, nullptr);
      registry->unregister(NETIO_DEFAULT_UDP_NAME, nullptr, nullptr);

      m_shmem_property.received_message_count_max = 1024 * 16;
      m_shmem_property.receive_buffer_size = 1024 * 1024 * 128 * 2;
      registry->register_component(
        NETIO_DEFAULT_SHMEM_NAME,
        SHMEMInterfaceFactory::get_interface(),
        &m_shmem_property._parent._parent,
        nullptr);

      registry->register_component(
        "dpde",
        DPDEDiscoveryFactory::get_interface(),
        &m_dpde_property._parent,
        nullptr);

      auto dp_qos = DDS_PARTICIPANT_QOS_DEFAULT;
      factory->get_default_participant_qos(dp_qos);

      dp_qos.participant_name.set_name("participant_name");

      dp_qos.discovery.discovery.name.set_name("dpde");

      dp_qos.discovery.initial_peers.maximum(1);
      dp_qos.discovery.initial_peers.length(1);
      *dp_qos.discovery.initial_peers.get_reference(0) =
        DDS_String_dup("_shmem://");

      dp_qos.transports.enabled_transports.ensure_length(2, 2);
      *dp_qos.transports.enabled_transports.get_reference(0) =
        DDS_String_dup(NETIO_DEFAULT_SHMEM_NAME);
      *dp_qos.transports.enabled_transports.get_reference(1) =
        DDS_String_dup(NETIO_DEFAULT_INTRA_NAME);

      dp_qos.discovery.enabled_transports.ensure_length(1, 1);
      *dp_qos.discovery.enabled_transports.get_reference(0) =
        DDS_String_dup("_shmem://");

      dp_qos.user_traffic.enabled_transports.ensure_length(2, 2);
      *dp_qos.user_traffic.enabled_transports.get_reference(0) =
        DDS_String_dup("_intra://");
      *dp_qos.user_traffic.enabled_transports.get_reference(1) =
        DDS_String_dup("_shmem://");

      dp_qos.resource_limits.local_writer_allocation = 500;
      dp_qos.resource_limits.local_reader_allocation = 500;
      dp_qos.resource_limits.local_publisher_allocation = 10;
      dp_qos.resource_limits.local_subscriber_allocation = 10;
      dp_qos.resource_limits.local_topic_allocation = 500;
      dp_qos.resource_limits.local_type_allocation = 500;
      dp_qos.resource_limits.remote_participant_allocation = 200;
      dp_qos.resource_limits.remote_writer_allocation = 200;
      dp_qos.resource_limits.remote_reader_allocation = 200;
      dp_qos.resource_limits.matching_writer_reader_pair_allocation = 200;
      dp_qos.resource_limits.matching_reader_writer_pair_allocation = 200;
      dp_qos.resource_limits.max_receive_ports = 200;
      dp_qos.resource_limits.max_destination_ports = 200;
      dp_qos.resource_limits.unbound_data_buffer_size = 65536;
      dp_qos.resource_limits.shmem_ref_transfer_mode_max_segments = 500;

      m_connext_dds_micro_participant = factory->create_participant(
        (DDS_DomainId_t)ec.dds_domain_id, dp_qos,
        nullptr /* listener */, DDS_STATUS_MASK_NONE);

      if (m_connext_dds_micro_participant == nullptr) {
        throw std::runtime_error("failed to create participant");
      }
    }
    return m_connext_dds_micro_participant;
  }

  /**
   * \brief Creates a new Connext DDS Micro publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void connext_dds_micro_publisher(
    const ExperimentConfiguration & ec,
    DDSPublisher * & publisher,
    DDS_DataWriterQos & dw_qos) const
  {
    auto participant = connext_DDS_micro_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);
    publisher = participant->create_publisher(
      DDS_PUBLISHER_QOS_DEFAULT, nullptr, DDS_STATUS_MASK_NONE);
    if (publisher == nullptr) {
      throw std::runtime_error("Pulisher is nullptr");
    }
    auto retcode = publisher->get_default_datawriter_qos(dw_qos);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to get default datawriter");
    }
  }

  /**
   * \brief Creates a new Connext DDS Micro subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void connext_dds_micro_subscriber(
    const ExperimentConfiguration & ec,
    DDSSubscriber * & subscriber,
    DDS_DataReaderQos & dr_qos) const
  {
    auto participant = connext_DDS_micro_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);
    subscriber = participant->create_subscriber(
      DDS_SUBSCRIBER_QOS_DEFAULT, nullptr,
      DDS_STATUS_MASK_NONE);
    if (subscriber == nullptr) {
      throw std::runtime_error("m_subscriber == nullptr");
    }
    auto retcode = subscriber->get_default_datareader_qos(dr_qos);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("failed get_default_datareader_qos");
    }
  }

private:
  RTIMicroDDSResourceManager()
  : m_connext_dds_micro_participant(nullptr) {}

  mutable DDSDomainParticipant * m_connext_dds_micro_participant;
  mutable NETIO_SHMEMInterfaceFactoryProperty m_shmem_property;
  mutable DPDE_DiscoveryPluginProperty m_dpde_property;
  mutable std::mutex m_global_mutex;
};

/**
 * \brief The plugin for Connext DDS Micro.
 * \tparam Topic The topic type to use.
 *
 * The code in here is derived from the C++ example in the Connext DDS Micro installation folder.
 */
template<class Topic>
class RTIMicroDDSPublisher : public Publisher
{
public:
  using DataType = typename Topic::ConnextDDSMicroType;
  using DataWriterType = typename DataType::DataWriter;
  using DataTypeSeq = typename DataType::Seq;

  explicit RTIMicroDDSPublisher(const ExperimentConfiguration & ec)
  : m_ec(ec),
    m_participant(RTIMicroDDSResourceManager::get().connext_DDS_micro_participant(ec)),
    m_datawriter(nullptr),
    m_topic(RTIMicroDDSTopicManager<Topic>::register_topic(m_participant, m_ec))
  {
    DDSPublisher * publisher;
    DDS_DataWriterQos dw_qos;
    RTIMicroDDSResourceManager::get().connext_dds_micro_publisher(ec, publisher, dw_qos);

    dw_qos.resource_limits.max_samples = 100;
    dw_qos.resource_limits.max_samples_per_instance = 100;
    dw_qos.resource_limits.max_instances = 1;

    ConnextDDSMicroQOSAdapter qos_adapter(m_ec.qos);
    qos_adapter.apply(dw_qos);

    m_datawriter = publisher->create_datawriter(
      m_topic,
      dw_qos, nullptr, DDS_STATUS_MASK_NONE);
    if (m_datawriter == nullptr) {
      throw std::runtime_error("Could not create datawriter");
    }

    m_typed_datawriter = DataWriterType::narrow(m_datawriter);
    if (m_typed_datawriter == nullptr) {
      throw std::runtime_error("failed datawriter narrow");
    }
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    init_data(m_data);
    init_msg(m_data, timestamp_provider, sample_id);
    auto retcode = m_typed_datawriter->write(m_data, DDS_HANDLE_NIL);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    DataType * sample;
    DDS_ReturnCode_t dds_rc = m_typed_datawriter->get_loan(sample);
    if (dds_rc != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to get a loan");
    }
    init_data(*sample);
    init_msg(*sample, timestamp_provider, sample_id);
    auto retcode = m_typed_datawriter->write(*sample, DDS_HANDLE_NIL);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

private:
  /**
  * \brief Initializes the frame_id field in data header.
  * This is the overloaded method which is called if data header has frame_id
  * \param data The data to publish.
  */
  template<typename T>
  static auto init_data(T & data)->decltype (data.header_.frame_id_, void ())
  {
    data.header_.frame_id_ = DDS_String_dup("frame_id");
    init_fields(data);
  }

  /**
  * \brief Initializes the frame_id_zc field in data header.
  * This is the overloaded method which is called if data header has frame_id
  * \param data The data to publish.
  */
  template<typename T>
  static auto init_data(T & data)->decltype (data.header_.frame_id_zc_, void ())
  {
    snprintf(data.header_.frame_id_zc_, 9U, "frame_id");
    init_fields(data);
  }

  /// Do nothing if frame_id not present
  static void init_data(...) {}

  /**
  * \brief Returns the size of array passed to it.
  * \param arr The array to compute the size of.
  */
  template<class T, size_t N>
  static constexpr size_t size(T (&)[N]) {return N;}

  /**
  * \brief Initializes the PointField array name field in data header.
  * This is the overloaded helper method which is called from init_data() if data has
  * PointField array in the payload
  * \param data The data to publish.
  */
  template<typename T>
  static auto init_fields(T & data)->decltype (data.fields_[0].name_, void ())
  {
    auto fields_size = size(data.fields_);
    for (uint8_t i = 0; i < fields_size; i++) {
      data.fields_[i].name_ = DDS_String_dup("name");
    }
  }

  /**
  * \brief Initializes the PointField array name field in data header for zero copy transfer.
  * This is the overloaded helper method which is called from init_data() if data has
  * PointField array in the payload
  * \param data The data to publish.
  */
  template<typename T>
  static auto init_fields(T & data)->decltype (data.fields_[0].name_zc_, void ())
  {
    auto fields_size = size(data.fields_);
    for (uint8_t i = 0; i < fields_size; i++) {
      snprintf(data.fields_[i].name_zc_, 5U, "name");
    }
  }

  /// Do nothing if PointField not present
  static void init_fields(...) {}

  const ExperimentConfiguration & m_ec;

  DDSDomainParticipant * m_participant;

  DDSDataWriter * m_datawriter;

  DataWriterType * m_typed_datawriter;

  DDSTopic * m_topic;

  DataType m_data;
};

/**
 * \brief The plugin for Connext DDS Micro.
 * \tparam Topic The topic type to use.
 *
 * The code in here is derived from the C++ example in the Connext DDS Micro installation folder.
 */
template<class Topic>
class RTIMicroDDSSubscriber : public Subscriber
{
public:
  using DataType = typename Topic::ConnextDDSMicroType;
  using DataReaderType = typename DataType::DataReader;
  using DataTypeSeq = typename DataType::Seq;

  explicit RTIMicroDDSSubscriber(const ExperimentConfiguration & ec)
  : m_ec(ec),
    m_participant(RTIMicroDDSResourceManager::get().connext_DDS_micro_participant(ec)),
    m_datareader(nullptr),
    m_typed_datareader(nullptr),
    m_topic(RTIMicroDDSTopicManager<Topic>::register_topic(m_participant, m_ec))
  {
    DDSSubscriber * subscriber = nullptr;
    DDS_DataReaderQos dr_qos;
    RTIMicroDDSResourceManager::get().connext_dds_micro_subscriber(ec, subscriber, dr_qos);

    dr_qos.resource_limits.max_samples = 100;
    dr_qos.resource_limits.max_instances = 1;
    dr_qos.resource_limits.max_samples_per_instance = 100;
    /* if there are more remote writers, you need to increase these limits */
    dr_qos.reader_resource_limits.max_remote_writers = 10;
    dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;

    ConnextDDSMicroQOSAdapter qos_adapter(m_ec.qos);
    qos_adapter.apply(dr_qos);

    /* Only DDS_DATA_AVAILABLE_STATUS supported currently */
    m_datareader = subscriber->create_datareader(
      m_topic,
      dr_qos,
      nullptr,
      DDS_STATUS_MASK_NONE);

    if (m_datareader == nullptr) {
      throw std::runtime_error("datareader == nullptr");
    }

    m_condition = m_datareader->get_statuscondition();
    m_condition->set_enabled_statuses(DDS_DATA_AVAILABLE_STATUS);
    m_waitset.attach_condition(m_condition);

    m_typed_datareader = DataReaderType::narrow(m_datareader);
    if (m_typed_datareader == nullptr) {
      throw std::runtime_error("m_typed_datareader == nullptr");
    }

    if (!m_condition_seq.ensure_length(2, 2)) {
      throw std::runtime_error("Error ensuring length of active_conditions_seq.");
    }
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    DDS_Duration_t wait_timeout = {15, 0};
    m_waitset.wait(m_condition_seq, wait_timeout);

    auto ret = m_typed_datareader->take(
      m_data_seq, m_sample_info_seq, DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE,
      DDS_ANY_INSTANCE_STATE);
    const auto received_time = now_int64_t();
    if (ret == DDS_RETCODE_OK) {
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

      m_typed_datareader->return_loan(
        m_data_seq,
        m_sample_info_seq);
    }
  }

private:
  const ExperimentConfiguration & m_ec;

  DDSDomainParticipant * m_participant;

  DDSDataReader * m_datareader;

  DDSWaitSet m_waitset;
  DDSStatusCondition * m_condition;
  DDSConditionSeq m_condition_seq;

  DataReaderType * m_typed_datareader;

  DataTypeSeq m_data_seq;
  DDS_SampleInfoSeq m_sample_info_seq;
  DDSTopic * m_topic;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__CONNEXTDDS_MICRO__CONNEXT_DDS_MICRO_COMMUNICATOR_HPP_
