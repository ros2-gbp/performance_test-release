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

#ifndef PERFORMANCE_TEST__PLUGINS__CYCLONEDDS__PLUGIN_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__CYCLONEDDS__PLUGIN_IMPL_HPP_

#include "performance_test/plugin/plugin.hpp"

#include <memory>
#include <string>

#include "cyclonedds/cyclonedds_communicator.hpp"
#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/runner_factory.hpp"
#include "performance_test/generated_messages/messages.hpp"
#include "performance_test/utilities/for_each.hpp"

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

private:
  template<class DataType>
  void register_pub_sub(PubSubRegistry & pub_sub_registry, const std::string & msg_name)
  {
    pub_sub_registry.register_pub_sub(
      "CycloneDDS",
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
             return std::make_unique<CycloneDDSPublisher<DataType>>(ec);
           };
  }

  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<CycloneDDSSubscriber<DataType>>(ec);
           };
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__CYCLONEDDS__PLUGIN_IMPL_HPP_
