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

#include "performance_test/experiment_metrics/subscriber_stats.hpp"

#include <chrono>
#include <exception>
#include <string>

#include "performance_test/experiment_metrics/analysis_result.hpp"

namespace performance_test
{
void SubscriberStats::on_message_received(
  const std::int64_t time_msg_sent_ns,
  const std::int64_t time_msg_received_ns,
  const std::uint64_t sample_id,
  const std::size_t data_type_size
)
{
  lock();
  verify_sample_chronological_order(time_msg_received_ns);
  update_lost_samples_counter(sample_id);
  add_latency_to_statistics(time_msg_sent_ns, time_msg_received_ns);
  increment_received();
  update_data_received(data_type_size);
  unlock();
}

void SubscriberStats::update_stats(std::chrono::duration<double> iteration_duration)
{
  lock();
  m_latency_stats_per_iteration = m_latency_stats;
  m_received_samples_per_iteration =
    static_cast<decltype(m_received_samples_per_iteration)>(
    static_cast<double>(m_received_sample_counter) /
    iteration_duration.count());
  m_received_data_bytes_per_iteration =
    static_cast<decltype(m_received_data_bytes_per_iteration)>(
    static_cast<double>(m_received_data_bytes) /
    iteration_duration.count());
  m_lost_samples_per_iteration =
    static_cast<decltype(m_lost_samples_per_iteration)>(
    static_cast<double>(m_num_lost_samples) /
    iteration_duration.count());

  m_latency_stats.clear();
  m_received_data_bytes = 0;
  m_received_sample_counter = 0;
  m_num_lost_samples = 0;
  unlock();
}

void SubscriberStats::populate_stats(AnalysisResult & results)
{
  lock();
  results.m_num_samples_received += m_received_samples_per_iteration;
  results.m_num_samples_lost += m_lost_samples_per_iteration;
  results.m_total_data_received += m_received_data_bytes_per_iteration;
  results.m_latency_stats.combine(m_latency_stats_per_iteration);
  unlock();
}

void SubscriberStats::lock()
{
  m_lock.lock();
}

void SubscriberStats::unlock()
{
  m_lock.unlock();
}

void SubscriberStats::verify_sample_chronological_order(std::int64_t time_ns_since_epoch)
{
  if (m_prev_timestamp_ns_since_epoch > time_ns_since_epoch) {
    throw std::runtime_error(
            "Data not consistent: received sample with not strictly older "
            "timestamp. Time diff: " +
            std::to_string(time_ns_since_epoch - m_prev_timestamp_ns_since_epoch) +
            " Data Time: " + std::to_string(time_ns_since_epoch));
  }
  m_prev_timestamp_ns_since_epoch = time_ns_since_epoch;
}

void SubscriberStats::update_lost_samples_counter(const std::uint64_t sample_id)
{
  // We can lose samples, but samples always arrive in the right order and
  // no duplicates exist.
  if (sample_id <= m_prev_sample_id) {
    throw std::runtime_error(
            "Data not consistent: received sample with not strictly greater id."
            " Received sample id: " +
            std::to_string(sample_id) +
            " Previous sample id: " + std::to_string(m_prev_sample_id));
  }
  m_num_lost_samples += sample_id - m_prev_sample_id - 1;
  m_prev_sample_id = sample_id;
}

void SubscriberStats::add_latency_to_statistics(
  const std::int64_t time_msg_sent_ns,
  const std::int64_t time_msg_received_ns)
{
  m_latency_stats.add_sample(time_msg_received_ns - time_msg_sent_ns);
}

void SubscriberStats::increment_received()
{
  m_received_sample_counter++;
}

void SubscriberStats::update_data_received(const std::size_t data_type_size)
{
  m_received_data_bytes = m_received_sample_counter * data_type_size;
}

}  // namespace performance_test
