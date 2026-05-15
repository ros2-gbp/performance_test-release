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

#ifndef PERFORMANCE_TEST__EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_

#include <chrono>

#include "performance_test/experiment_metrics/analysis_result.hpp"
#include "performance_test/utilities/spin_lock.hpp"

namespace performance_test
{
struct PublisherStats
{
  std::uint64_t next_sample_id();
  void on_message_sent();
  void update_stats(std::chrono::duration<double> iteration_duration);
  void populate_stats(AnalysisResult & results);

private:
  void lock();
  void unlock();
  void increment_sent();

  std::uint64_t m_sent_sample_counter{};
  std::uint64_t m_sent_samples_per_iteration{};
  std::uint64_t m_prev_sample_id{};
  SpinLock m_lock;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_
