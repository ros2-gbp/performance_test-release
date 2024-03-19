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

#ifndef PERFORMANCE_TEST__EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_
#define PERFORMANCE_TEST__EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_

#include <memory>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/subscriber_stats.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "performance_test/utilities/memory_checker.hpp"

namespace performance_test
{
class SubscriberTask
{
public:
  SubscriberTask(
    const ExperimentConfiguration & ec,
    SubscriberStats & stats,
    std::unique_ptr<Subscriber> && sub);

  SubscriberTask & operator=(const SubscriberTask &) = delete;
  SubscriberTask(const SubscriberTask &) = delete;

  void run();

  void take();

private:
  SubscriberStats & m_stats;
  std::unique_ptr<Subscriber> m_sub;
  MemoryChecker m_memory_checker;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_
