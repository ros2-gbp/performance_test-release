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

#include "performance_test/execution_tasks/subscriber_task.hpp"

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_factory.hpp"
#include "performance_test/experiment_metrics/subscriber_stats.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{

SubscriberTask::SubscriberTask(
  const ExperimentConfiguration & ec,
  SubscriberStats & stats)
: m_stats(stats),
  m_sub(PubSubFactory::get().create_subscriber(ec)),
  m_memory_checker(ec) {}

void SubscriberTask::prepare()
{
  m_sub->prepare();
}

void SubscriberTask::run()
{
  m_sub->update_subscription(m_stats);
  m_memory_checker.enable_memory_tools_checker();
}

void SubscriberTask::take()
{
  m_sub->take(m_stats);
  m_memory_checker.enable_memory_tools_checker();
}

}  // namespace performance_test
