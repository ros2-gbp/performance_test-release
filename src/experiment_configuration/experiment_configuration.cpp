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

#include "performance_test/experiment_configuration/experiment_configuration.hpp"

#include <tclap/CmdLine.h>

#include <csignal>
#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <vector>
#include <memory>
#include <sole/sole.hpp>

#include "performance_test/plugin/plugin_singleton.hpp"
#include "performance_test/utilities/version.hpp"

namespace performance_test
{

ExperimentConfiguration::ExperimentConfiguration()
: id(sole::uuid4().str()) {}

std::chrono::duration<double> ExperimentConfiguration::period() const
{
  return std::chrono::duration<double>(1.0 / rate);
}

std::chrono::nanoseconds ExperimentConfiguration::period_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period());
}

std::string ExperimentConfiguration::pub_topic_postfix() const
{
  std::string fix;
  if (roundtrip_mode == RoundTripMode::MAIN) {
    fix = "main";
  } else if (roundtrip_mode == RoundTripMode::RELAY) {
    fix = "relay";
  }
  return fix;
}

std::string ExperimentConfiguration::sub_topic_postfix() const
{
  std::string fix;
  if (roundtrip_mode == RoundTripMode::MAIN) {
    fix = "relay";
  } else if (roundtrip_mode == RoundTripMode::RELAY) {
    fix = "main";
  }
  return fix;
}

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e)
{
  stream <<
    "Experiment id: " << e.id <<
    "\nPerformance Test Version: " << version() <<
    "\nLogfile name: " << e.output_configuration.logfile_path <<
    "\nCommunicator: " << e.communicator <<
    "\nDDS domain id: " << e.dds_domain_id <<
    "\nQOS: " << e.qos <<
    "\nPublishing rate: " << e.rate <<
    "\nTopic name: " << e.topic_name <<
    "\nMsg name: " << e.msg_name <<
    "\nMaximum runtime (sec): " << e.max_runtime <<
    "\nNumber of publishers: " << e.number_of_publishers <<
    "\nNumber of subscribers: " << e.number_of_subscribers <<
    "\nMemory check enabled: " << e.check_memory <<
    "\nWith security: " << e.with_security <<
    "\nZero copy transfer: " << e.is_zero_copy_transfer <<
    "\nUnbounded message size: " << e.unbounded_msg_size <<
    "\nRoundtrip Mode: " << e.roundtrip_mode <<
    "\nIgnore seconds from beginning: " << e.rows_to_ignore;
  for (const auto & kvp : PluginSingleton::get()->extra_log_info()) {
    stream << std::endl << kvp.first << ": " << kvp.second;
  }
  return stream;
}

}  // namespace performance_test
