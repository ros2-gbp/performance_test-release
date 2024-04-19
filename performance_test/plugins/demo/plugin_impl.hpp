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

#ifndef PERFORMANCE_TEST__PLUGINS__DEMO__PLUGIN_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__DEMO__PLUGIN_IMPL_HPP_

#include "performance_test/plugin/plugin.hpp"

#include <memory>
#include <map>
#include <string>

#include "performance_test/plugins/demo/publisher_impl.hpp"
#include "performance_test/plugins/demo/subscriber_impl.hpp"
#include "performance_test/plugins/demo/msg/array16k.hpp"
#include "performance_test/plugins/demo/msg/array1m.hpp"
#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_registry.hpp"
#include "performance_test/experiment_execution/runner_registry.hpp"

namespace performance_test
{
class PluginImpl : public Plugin
{
public:
  void register_pub_sub(PubSubRegistry & pub_sub_registry) override
  {
    // The perf_test application will create publishers and subscribers based on
    // the selected communicator and message. Most plugins only have one type of
    // publisher and subscriber. See the ros2 Plugin for an example of multiple
    // subscriber types.

    // Message types will usually be generated from .idl files, but that is beyond
    // the scope of this demo Plugin. An example of message generation with bazel
    // can be found at https://github.com/ApexAI/rules_ros

    // perf_test -c demo -m Array16k
    pub_sub_registry.register_pub_sub(
      "demo",
      "Array16k",
      pub_producer<msg::Array16k>(),
      sub_producer<msg::Array16k>()
    );
    pub_sub_registry.register_pub_sub(
      "demo",
      "Array1m",
      pub_producer<msg::Array1m>(),
      sub_producer<msg::Array1m>()
    );
  }

  void register_custom_runners(RunnerRegistry & runner_registry) override
  {
    // The perf_test application provides some default runners which should be
    // sufficient for most experiments. Therefore, most Plugins will not need
    // to register any custom runners. Custom runners can be useful for running
    // an experiment with a specific execution strategy, such as a special
    // thread manager for pubs/subs. See the Apex.OS plugin for an example.
  }

  void global_setup(const ExperimentConfiguration & ec) override
  {
    // This is called after the CLI args are parsed, but before any pubs/subs
    // are created. This is the place to initialize any global context or shared
    // resources required by the pubs/subs.
    if (ec.number_of_publishers > 1 || ec.number_of_subscribers > 1) {
      throw std::runtime_error("This plugin only supports 1 publisher and 1 subscriber");
    }
    if (ec.is_zero_copy_transfer) {
      throw std::runtime_error("This plugin does not support zero-copy transfer");
    }
  }

  void global_teardown(const ExperimentConfiguration & ec) override
  {
    // This is called at the end of the experiment. This is the place to clean up
    // the resources initialized or allocated in global_setup().
  }

  std::map<std::string, std::string> extra_log_info() override
  {
    // The key/value pairs in this map are added directly to the log output,
    // as if they are properties of the ExperimentConfiguration.
    // There are no required values. The map could even be empty.
    std::map<std::string, std::string> m;
    m["plugin_name"] = "demo";
    m["plugin_version"] = "1.0.0";
    return m;
  }

  bool exit_requested() override
  {
    // The perf_test application periodically checks for SIGINT to gracefully
    // terminate the experiment. In addition, it also calls this method to check
    // if the Plugin needs to shut down the experiment.
    return false;
  }

private:
  template<class DataType>
  PubSubRegistry::PublisherProducer pub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Publisher>
           {
             return std::make_unique<PublisherImpl<DataType>>(ec);
           };
  }

  template<class DataType>
  PubSubRegistry::SubscriberProducer sub_producer()
  {
    return [](const ExperimentConfiguration & ec) -> std::unique_ptr<Subscriber>
           {
             return std::make_unique<SubscriberImpl<DataType>>(ec);
           };
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__DEMO__PLUGIN_IMPL_HPP_
