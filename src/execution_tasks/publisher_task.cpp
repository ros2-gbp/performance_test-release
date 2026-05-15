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

#include "performance_test/execution_tasks/publisher_task.hpp"

#include <thread>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_factory.hpp"
#include "performance_test/experiment_metrics/publisher_stats.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/utilities/memory_checker.hpp"
#include "performance_test/utilities/perf_clock.hpp"

namespace performance_test
{

PublisherTask::PublisherTask(
  const ExperimentConfiguration & ec,
  PublisherStats & stats)
: m_ec(ec),
  m_stats(stats),
  m_pub(PubSubFactory::get().create_publisher(ec)),
  m_time_between_publish(ec.period_ns()),
  m_loop_counter(0),
  m_memory_checker(ec) {}

void PublisherTask::prepare()
{
  m_pub->prepare();
}

void PublisherTask::run()
{
  if (m_first_run.time_since_epoch().count() == 0) {
    m_first_run = perf_clock::now();
  }

  perf_clock::time_point next_run = m_first_run + m_time_between_publish * m_loop_counter++;
  std::this_thread::sleep_until(next_run);

  if (m_ec.use_loaned_samples) {
    m_pub->publish_loaned(m_timestamp_provider, m_stats.next_sample_id());
  } else {
    m_pub->publish_copy(m_timestamp_provider, m_stats.next_sample_id());
  }
  m_stats.on_message_sent();

  m_memory_checker.enable_memory_tools_checker();
}

}  // namespace performance_test
