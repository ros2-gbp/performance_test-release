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

#include "performance_test/experiment_metrics/publisher_stats.hpp"

#include <chrono>

#include "performance_test/experiment_metrics/analysis_result.hpp"

namespace performance_test
{
std::uint64_t PublisherStats::next_sample_id()
{
  // Pre-increment, so the first sample ID is 1.
  // If a sample ID is ever 0, then the sample has not been initialized.
  return ++m_prev_sample_id;
}

void PublisherStats::on_message_sent()
{
  lock();
  increment_sent();
  unlock();
}

void PublisherStats::update_stats(std::chrono::duration<double> iteration_duration)
{
  lock();
  m_sent_samples_per_iteration =
    static_cast<decltype(m_sent_samples_per_iteration)>(
    static_cast<double>(m_sent_sample_counter) /
    iteration_duration.count());
  m_sent_sample_counter = 0;
  unlock();
}

void PublisherStats::populate_stats(AnalysisResult & results)
{
  lock();
  results.m_num_samples_sent += m_sent_samples_per_iteration;
  unlock();
}

void PublisherStats::lock()
{
  m_lock.lock();
}

void PublisherStats::unlock()
{
  m_lock.unlock();
}

void PublisherStats::increment_sent()
{
  m_sent_sample_counter++;
}

}  // namespace performance_test
