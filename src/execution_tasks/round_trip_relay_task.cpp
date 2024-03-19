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

#include "performance_test/execution_tasks/round_trip_relay_task.hpp"

#include <memory>
#include <utility>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{

RoundTripRelayTask::RoundTripRelayTask(
  const ExperimentConfiguration & ec,
  std::unique_ptr<Publisher> && pub,
  std::unique_ptr<Subscriber> && sub)
: m_pub(std::move(pub)),
  m_sub(std::move(sub)),
  m_memory_checker(ec) {}

void RoundTripRelayTask::run()
{
  m_sub->update_subscription(*this);
  m_memory_checker.enable_memory_tools_checker();
}

void RoundTripRelayTask::on_message_received(
  const std::int64_t time_msg_sent_ns,
  const std::int64_t,
  const std::uint64_t sample_id,
  const std::size_t
)
{
  RoundtripTimestampProvider ts(time_msg_sent_ns);
  m_pub->publish_copy(ts, sample_id);
}

}  // namespace performance_test
