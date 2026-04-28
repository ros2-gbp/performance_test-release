// Copyright 2022-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_
#define PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_

#include <memory>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/utilities/memory_checker.hpp"
#include "performance_test/utilities/perf_clock.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{
class PublisherTask
{
public:
  PublisherTask(
    const ExperimentConfiguration & ec,
    PublisherStats & stats);

  PublisherTask & operator=(const PublisherTask &) = delete;
  PublisherTask(const PublisherTask &) = delete;

  void prepare();

  void run();

private:
  const ExperimentConfiguration & m_ec;
  PublisherStats & m_stats;
  std::unique_ptr<Publisher> m_pub;
  const std::chrono::nanoseconds m_time_between_publish;
  perf_clock::time_point m_first_run;
  std::size_t m_loop_counter;
  MemoryChecker m_memory_checker;
  PublisherTimestampProvider m_timestamp_provider;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXECUTION_TASKS__PUBLISHER_TASK_HPP_
