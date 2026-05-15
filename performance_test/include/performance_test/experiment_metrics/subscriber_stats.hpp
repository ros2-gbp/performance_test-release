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

#ifndef PERFORMANCE_TEST__EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_

#include <chrono>

#include "performance_test/experiment_metrics/analysis_result.hpp"
#include "performance_test/experiment_metrics/message_received_listener.hpp"
#include "performance_test/utilities/sample_statistics.hpp"
#include "performance_test/utilities/spin_lock.hpp"

namespace performance_test
{
class SubscriberStats : public MessageReceivedListener
{
public:
  void on_message_received(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t time_msg_received_ns,
    const std::uint64_t sample_id,
    const std::size_t data_type_size
  ) override;
  void update_stats(std::chrono::duration<double> iteration_duration);
  void populate_stats(AnalysisResult & results);

private:
  void lock();
  void unlock();
  void verify_sample_chronological_order(std::int64_t time_ns_since_epoch);
  void update_lost_samples_counter(const std::uint64_t sample_id);
  void add_latency_to_statistics(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t time_msg_received_ns);
  void increment_received();
  void update_data_received(const std::size_t data_type_size);

  std::int64_t m_prev_timestamp_ns_since_epoch{};
  std::uint64_t m_prev_sample_id{};

  SampleStatistics<std::int64_t> m_latency_stats;
  std::uint64_t m_received_sample_counter{};
  std::size_t m_received_data_bytes{};
  std::uint64_t m_num_lost_samples{};

  SampleStatistics<std::int64_t> m_latency_stats_per_iteration;
  std::size_t m_received_data_bytes_per_iteration{};
  std::uint64_t m_received_samples_per_iteration{};
  std::uint64_t m_lost_samples_per_iteration{};

  SpinLock m_lock;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_
