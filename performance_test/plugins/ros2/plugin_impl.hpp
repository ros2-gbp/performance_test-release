// Copyright 2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__ROS2__PLUGIN_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__ROS2__PLUGIN_IMPL_HPP_

#include "performance_test/plugin/plugin.hpp"

#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/generated_messages/messages.hpp"
#include "performance_test/utilities/for_each.hpp"
#include "rclcpp_common/rclcpp_publisher.hpp"

#if defined(PERFORMANCE_TEST_RCLCPP_STE_ENABLED) || defined(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED)
#include "ros2/rclcpp_callback_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
#include "ros2/rclcpp_waitset_communicator.hpp"
#endif

namespace performance_test
{
class PluginImpl : public Plugin
{
public:
  void register_pub_sub(PubSubRegistry & pub_sub_registry) override
  {
    performance_test::for_each(
      messages::MessageTypeList(),
      [&](const auto & msg_type) {
        using T = std::remove_cv_t<std::remove_reference_t<decltype(msg_type)>>;
        register_pub_sub<T>(pub_sub_registry, T::msg_name());
      });
  }

  void global_setup(const ExperimentConfiguration & ec) override
  {
    if (ec.use_shared_memory) {
      enable_shared_memory();
    }

#if defined(PERFORMANCE_TEST_ROS2_DASHING) || defined(PERFORMANCE_TEST_ROS2_ELOQUENT)
    rclcpp::contexts::default_context::get_global_default_context()->init(
      ec.argc, ec.argv, rclcpp::InitOptions{});
#elif defined(PERFORMANCE_TEST_ROS2_FOXY) || defined(PERFORMANCE_TEST_ROS2_GALACTIC)
    rclcpp::contexts::get_global_default_context()->init(
      ec.argc, ec.argv, rclcpp::InitOptions{});
#else
    rclcpp::init(ec.argc, ec.argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
#endif
  }

  std::map<std::string, std::string> extra_log_info() override
  {
    std::map<std::string, std::string> m;
    m["rmw_implementation"] = rmw_get_implementation_identifier();
    return m;
  }

  bool exit_requested() override
  {
    return !rclcpp::ok();
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;

  template<class DataType>
  void register_pub_sub(PubSubRegistry & pub_sub_registry, const std::string & msg_name)
  {
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
    pub_sub_registry.register_pub_sub(
      "rclcpp-single-threaded-executor",
      msg_name,
      pub_producer<DataType>(),
      sub_ste_producer<DataType>()
    );
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
    pub_sub_registry.register_pub_sub(
      "rclcpp-static-single-threaded-executor",
      msg_name,
      pub_producer<DataType>(),
      sub_sste_producer<DataType>()
    );
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
    pub_sub_registry.register_pub_sub(
      "rclcpp-waitset",
      msg_name,
      pub_producer<DataType>(),
      sub_waitset_producer<DataType>()
    );
#endif
  }

  template<class DataType>
  PubSubRegistry::PublisherProducer pub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Publisher>
           {
             return std::make_unique<RclcppPublisher<DataType>>(ec);
           };
  }

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_ste_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<RclcppSingleThreadedExecutorSubscriber<DataType>>(ec);
           };
  }
#endif

#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_sste_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<RclcppStaticSingleThreadedExecutorSubscriber<DataType>>(ec);
           };
  }
#endif

#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_waitset_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<RclcppWaitsetSubscriber<DataType>>(ec);
           };
  }
#endif

  static void enable_shared_memory()
  {
    std::string rmw_implementation = rmw_get_implementation_identifier();
    if (rmw_implementation == "rmw_cyclonedds_cpp") {
      enable_shared_memory_cyclonedds();
    } else if (rmw_implementation == "rmw_fastrtps_cpp") {
      enable_shared_memory_fastrtps();
    } else {
      throw std::runtime_error("Shared memory is not supported for " + rmw_implementation);
    }
  }

  static void enable_shared_memory_cyclonedds()
  {
    const std::string config_string =
      "<CycloneDDS>\n"
      "    <Domain>\n"
      "        <SharedMemory>\n"
      "            <Enable>true</Enable>\n"
      "            <LogLevel>verbose</LogLevel>\n"
      "        </SharedMemory>\n"
      "    </Domain>\n"
      "</CycloneDDS>\n";
    setenv("CYCLONEDDS_URI", config_string.c_str(), 1);
  }

  static void enable_shared_memory_fastrtps()
  {
    const std::string config_string =
      "<data_writer profile_name=\"topic_name\">\n"
      "    <qos>\n"
      "        <data_sharing>\n"
      "            <kind>AUTOMATIC</kind>\n"
      "        </data_sharing>\n"
      "    </qos>\n"
      "</data_writer>\n"
      "<data_reader profile_name=\"topic_name\">\n"
      "    <qos>\n"
      "        <data_sharing>\n"
      "            <kind>AUTOMATIC</kind>\n"
      "        </data_sharing>\n"
      "    </qos>\n"
      "</data_reader>\n";
    const std::string shmem_file_path = "/tmp/perf_test_shmem.xml";
    std::ofstream temp_file(shmem_file_path, std::ios_base::app);
    if (temp_file.is_open()) {
      temp_file << config_string;
      temp_file.close();
    } else {
      throw std::runtime_error("Failed to open " + shmem_file_path);
    }
    setenv("FASTDDS_DEFAULT_PROFILES_FILE", shmem_file_path.c_str(), 1);
    setenv("RMW_FASTRTPS_USE_QOS_FROM_XML", "1", 1);
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__ROS2__PLUGIN_IMPL_HPP_
