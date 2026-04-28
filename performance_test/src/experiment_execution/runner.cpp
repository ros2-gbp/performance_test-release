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

#include "performance_test/experiment_execution/runner.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/analysis_result.hpp"
#include "performance_test/outputs/output_factory.hpp"
#include "performance_test/plugin/plugin_singleton.hpp"
#include "performance_test/utilities/exit_request_handler.hpp"

namespace performance_test
{
Runner::Runner(const ExperimentConfiguration & ec)
: m_ec(ec),
  m_pub_stats(ec.number_of_publishers),
  m_sub_stats(ec.number_of_subscribers),
  m_outputs(OutputFactory::get(ec.output_configuration))
{
  for (const auto & output : m_outputs) {
    output->open(ec);
  }
}

Runner::~Runner()
{
  for (const auto & output : m_outputs) {
    output->close();
  }
}

void Runner::run()
{
  m_running = true;

  run_pubs_and_subs();

  const auto experiment_start = perf_clock::now();
  perf_clock::time_point last_measurement_time{};

  while (!check_exit(experiment_start)) {
    const auto loop_start = perf_clock::now();
    const auto time_between_two_measurements =
      loop_start - last_measurement_time;

    // Collect measurements every second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (auto & pub : m_pub_stats) {
      pub.update_stats(time_between_two_measurements);
    }
    for (auto & sub : m_sub_stats) {
      sub.update_stats(time_between_two_measurements);
    }

    if (ignore_first_seconds_of_experiment(experiment_start)) {
      AnalysisResult results;
      results.m_experiment_start = loop_start - experiment_start;
      results.m_time_between_two_measurements =
        time_between_two_measurements;
      results.m_cpu_info = cpu_usage_tracker.get_cpu_usage();

      for (auto & pub : m_pub_stats) {
        pub.populate_stats(results);
      }
      for (auto & sub : m_sub_stats) {
        sub.populate_stats(results);
      }

      for (const auto & output : m_outputs) {
        output->update(results);
      }
    }
    last_measurement_time = loop_start;
  }
  m_running = false;
}

bool Runner::ignore_first_seconds_of_experiment(
  const perf_clock::time_point & experiment_start)
{
  const auto time_elapsed = perf_clock::now() - experiment_start;
  auto time_elapsed_s =
    std::chrono::duration_cast<std::chrono::seconds>(time_elapsed).count();

  return time_elapsed_s > m_ec.rows_to_ignore;
}

bool Runner::check_exit(perf_clock::time_point experiment_start)
{
  if (ExitRequestHandler::get().exit_requested() || PluginSingleton::get()->exit_requested()) {
    std::cout << "Caught signal. Exiting." << std::endl;
    return true;
  }

  if (m_ec.max_runtime == 0) {
    // Run forever,
    return false;
  }

  const double runtime_sec =
    std::chrono::duration<double>(perf_clock::now() - experiment_start)
    .count();

  if (runtime_sec > static_cast<double>(m_ec.max_runtime)) {
    std::cout << "Maximum runtime reached. Exiting." << std::endl;
    return true;
  } else {
    return false;
  }
}
}  // namespace performance_test
