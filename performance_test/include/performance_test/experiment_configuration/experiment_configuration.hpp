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

#ifndef PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_

#include <atomic>
#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <chrono>

#include "performance_test/experiment_configuration/output_configuration.hpp"
#include "performance_test/experiment_configuration/qos_abstraction.hpp"
#include "performance_test/experiment_configuration/real_time_configuration.hpp"
#include "performance_test/experiment_configuration/round_trip_mode.hpp"

namespace performance_test
{

/**
 * \brief Represents the configuration of an experiment.
 *
 * This experiment configuration could be created from various sources. At the
 * moment, only configuration by command line arguments are supported.
 */
struct ExperimentConfiguration
{
  ExperimentConfiguration();

  const std::string id;
  std::string communicator;
  std::string execution_strategy;
  uint32_t dds_domain_id;
  QOSAbstraction qos;
  uint32_t rate;
  std::string topic_name;
  std::string msg_name;
  size_t unbounded_msg_size;

  uint64_t max_runtime;
  uint32_t rows_to_ignore;
  uint32_t number_of_publishers;
  uint32_t number_of_subscribers;
  uint32_t expected_num_pubs;
  uint32_t expected_num_subs;
  std::chrono::seconds wait_for_matched_timeout;
  bool check_memory;
  RealTimeConfiguration rt_config;
  bool with_security;
  bool use_shared_memory;
  bool use_loaned_samples;
  bool prevent_cpu_idle;

  RoundTripMode roundtrip_mode;
  OutputConfiguration output_configuration;

  /// @brief In case any plugins need extra CLI args
  int argc;
  char ** argv;

  /// \returns Returns the inverse of the configured publishing rate.
  std::chrono::duration<double> period() const;
  /// \returns Returns the inverse of the configured publishing rate, in nanoseconds.
  std::chrono::nanoseconds period_ns() const;
  std::string pub_topic_postfix() const;
  std::string sub_topic_postfix() const;
};

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e);
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
