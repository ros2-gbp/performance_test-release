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

#ifndef PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_FACTORY_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_FACTORY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/runner_registry.hpp"
#include "performance_test/experiment_execution/runner.hpp"

namespace performance_test
{
class RunnerFactory : public RunnerRegistry
{
public:
  static RunnerFactory & get()
  {
    static RunnerFactory instance;

    return instance;
  }

  RunnerFactory(RunnerFactory const &) = delete;
  RunnerFactory(RunnerFactory &&) = delete;

  RunnerFactory & operator=(RunnerFactory const &) = delete;
  RunnerFactory & operator=(RunnerFactory &&) = delete;

  void register_runner(
    const ExecutionStrategy & execution_strategy,
    RunnerProducer runner_producer) override;
  std::vector<std::string> supported_execution_strategies() const;
  std::unique_ptr<Runner> create_runner(const ExperimentConfiguration & ec) const;

private:
  RunnerFactory();

  std::map<ExecutionStrategy, RunnerProducer> m_producers;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_EXECUTION__RUNNER_FACTORY_HPP_
