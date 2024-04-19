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

#ifndef PERFORMANCE_TEST__PLUGIN__PLUGIN_HPP_
#define PERFORMANCE_TEST__PLUGIN__PLUGIN_HPP_

#include <map>
#include <string>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_registry.hpp"
#include "performance_test/experiment_execution/runner_registry.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{
class Plugin
{
public:
  virtual ~Plugin() = default;

  /// @brief Register the Plugin's Publishers and Subscribers
  ///
  /// Each Plugin must implement this method.
  /// When the Plugin invokes pub_sub_registry.register_pub_sub(), it instructs
  /// the performance_test core how to construct a Publisher and Subscriber
  /// for the selected message type.
  /// @param pub_sub_registry
  virtual void register_pub_sub(PubSubRegistry & pub_sub_registry) = 0;

  /// @brief Register the Plugin's custom Runners.
  ///
  /// Most Plugins will not need to implement this method. The built-in Runners
  /// will be sufficient for most performance experiments.
  /// When the Plugin invokes runner_registry.register_runner(), it instructs
  /// the performance_test core how to construct additional Runners beyond the
  /// built-in Runners.
  /// @param runner_registry
  virtual void register_custom_runners(RunnerRegistry & runner_registry) {}

  /// @brief Set up any global context or shared state necessary for the Pubs and Subs.
  ///
  /// This method is called before creating the Runner, the Publishers, and the Subscribers.
  /// If there is any global setup to perform, or any shared state to initialize, then
  /// the Plugin should override this method and perform that setup here.
  /// @param ec
  virtual void global_setup(const ExperimentConfiguration & ec) {}

  /// @brief  Tear down any global context or shared state.
  ///
  /// This method is called after the experiment is finished. If the Plugin needs to reset
  /// any global state, remove temporary files, etc., then the Plugin should override this
  /// method and perform that teardown here.
  /// @param ec
  virtual void global_teardown(const ExperimentConfiguration & ec) {}

  /// @brief Provide Plugin-specific information for the logs.
  ///
  /// When performance_test writes a log file, this extra information is added to the
  /// section of the log file that corresponds to the ExperimentConfiguration.
  /// A Plugin should override this if, for example, it is configured by environment
  /// variables beyond the scope of ExperimentConfiguration. Those environment
  /// variables could be included in the logs, so there is no question about the
  /// experiment's full configuration.
  /// @return Plugin-specific information to log.
  virtual std::map<std::string, std::string> extra_log_info() {return {};}

  /// @brief Allow the Plugin to request that the experiment end early.
  ///
  /// If a Plugin throws an unhandled exception, then it is unlikely that any data
  /// will be captured in the logs. This is fine for early detection of configuration
  /// errors, but not ideal if something goes wrong halfway through the experiment.
  /// This method allows the Plugin to request a clean exit when the Runner finishes
  /// its current loop.
  /// @return Whether the Plugin has requested the experiment to exit.
  virtual bool exit_requested() {return false;}
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__PLUGIN_HPP_
