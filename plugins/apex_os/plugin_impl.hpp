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

#ifndef PERFORMANCE_TEST__PLUGINS__APEX_OS__PLUGIN_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__APEX_OS__PLUGIN_IMPL_HPP_

#include "performance_test/plugin/plugin.hpp"

#include <memory>
#include <map>
#include <string>

#include <cyclone_dds_vendor/dds.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <settings/inspect.hpp>
#include <settings/repository.hpp>

#include "apex_os/apex_os_polling_subscription_communicator.hpp"
#include "apex_os/apex_os_runner.hpp"
#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/runner_factory.hpp"
#include "performance_test/generated_messages/messages.hpp"
#include "performance_test/utilities/for_each.hpp"
#include "rclcpp_common/rclcpp_publisher.hpp"

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

  void register_custom_runners(RunnerRegistry & runner_registry) override {
    runner_registry.register_runner(
      "APEX_SINGLE_EXECUTOR",
      [](const ExperimentConfiguration & ec)->std::unique_ptr<Runner> {
          return std::make_unique<ApexOsSingleExecutorRunner>(ec);
      });
    runner_registry.register_runner(
      "APEX_EXECUTOR_PER_COMMUNICATOR",
      [](const ExperimentConfiguration & ec)->std::unique_ptr<Runner> {
          return std::make_unique<ApexOsExecutorPerCommunicatorRunner>(ec);
      });
    runner_registry.register_runner(
      "APEX_CHAIN",
      [](const ExperimentConfiguration & ec)->std::unique_ptr<Runner> {
          return std::make_unique<ApexOsSingleExecutorChainRunner>(ec);
      });
  }

  void global_setup(const ExperimentConfiguration & ec) override
  {
    rclcpp::init(ec.argc, ec.argv, rclcpp::InitOptions{}, false);
  }

  std::map<std::string, std::string> extra_log_info() override
  {
    std::map<std::string, std::string> m;
    m["rmw_implementation"] = rmw_get_implementation_identifier();
#ifdef DDSCXX_HAS_SHM
    bool shm = apex::settings::inspect::get_or_default<bool>(
        apex::settings::repository::get(), "domain/shared_memory/enable", false);
#else
    bool shm = false;
#endif
    m["is_shared_memory_transfer"] = shm ? "true" : "false";
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
    pub_sub_registry.register_pub_sub(
      "ApexOSPollingSubscription",
      msg_name,
      pub_producer<DataType>(),
      sub_producer<DataType>()
    );
  }

  template<class DataType>
  PubSubRegistry::PublisherProducer pub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Publisher>
           {
             return std::make_unique<RclcppPublisher<DataType>>(ec);
           };
  }

  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<ApexOSPollingSubscriptionSubscriber<DataType>>(ec);
           };
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__APEX_OS__PLUGIN_IMPL_HPP_
