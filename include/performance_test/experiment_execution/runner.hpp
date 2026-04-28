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

#ifndef PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/experiment_metrics/subscriber_stats.hpp"
#include "performance_test/outputs/output.hpp"
#include "performance_test/utilities/cpu_usage_tracker.hpp"
#include "performance_test/utilities/exit_request_handler.hpp"

namespace performance_test
{

class Runner
{
public:
  explicit Runner(const ExperimentConfiguration & ec);
  virtual ~Runner();

  void run();

protected:
  virtual void run_pubs_and_subs() = 0;
  const ExperimentConfiguration & m_ec;
  std::vector<PublisherStats> m_pub_stats;
  std::vector<SubscriberStats> m_sub_stats;
  std::atomic<bool> m_running{false};

private:
  bool ignore_first_seconds_of_experiment(
    const perf_clock::time_point & experiment_start);

  bool check_exit(perf_clock::time_point experiment_start);

  std::vector<std::shared_ptr<Output>> m_outputs;
  CPUsageTracker cpu_usage_tracker;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_HPP_
