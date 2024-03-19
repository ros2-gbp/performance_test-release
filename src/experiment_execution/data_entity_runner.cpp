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

#include "performance_test/experiment_execution/data_entity_runner.hpp"

#include <memory>

#include "performance_test/execution_tasks/publisher_task.hpp"
#include "performance_test/execution_tasks/subscriber_task.hpp"
#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_factory.hpp"
#include "performance_test/experiment_execution/runner.hpp"

namespace performance_test
{
DataEntityRunner::DataEntityRunner(const ExperimentConfiguration & ec)
: Runner(ec)
{
  for (uint32_t i = 0; i < m_ec.number_of_subscribers; ++i) {
    m_subs.push_back(
      std::make_shared<SubscriberTask>(
        ec,
        m_sub_stats.at(i),
        PubSubFactory::get().create_subscriber(ec)));
  }
  for (uint32_t i = 0; i < m_ec.number_of_publishers; ++i) {
    m_pubs.push_back(
      std::make_shared<PublisherTask>(
        ec,
        m_pub_stats.at(i),
        PubSubFactory::get().create_publisher(ec)));
  }
}

DataEntityRunner::~DataEntityRunner() {}

}  // namespace performance_test
