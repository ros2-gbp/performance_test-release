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

#include "performance_test/experiment_execution/runner_factory.hpp"

#include <memory>
#include <string>
#include <vector>

#include "performance_test/experiment_execution/inter_thread_runner.hpp"
#include "performance_test/experiment_execution/intra_thread_runner.hpp"
#include "performance_test/experiment_execution/round_trip_main_runner.hpp"
#include "performance_test/experiment_execution/round_trip_relay_runner.hpp"

namespace performance_test
{
RunnerFactory::RunnerFactory()
{
  register_runner(
    "INTER_THREAD",
    [](const ExperimentConfiguration & ec) -> std::unique_ptr<Runner> {
      switch (ec.roundtrip_mode) {
        case RoundTripMode::NONE:
          return std::make_unique<InterThreadRunner>(ec);
        case RoundTripMode::MAIN:
          return std::make_unique<RoundTripMainRunner>(ec);
        case RoundTripMode::RELAY:
          return std::make_unique<RoundTripRelayRunner>(ec);
        default:
          throw std::invalid_argument("Invalid round trip mode");
      }
    });
  register_runner(
    "INTRA_THREAD",
    [](const ExperimentConfiguration & ec) -> std::unique_ptr<Runner> {
      return std::make_unique<IntraThreadRunner>(ec);
    });
}

void RunnerFactory::register_runner(
  const ExecutionStrategy & execution_strategy,
  RunnerProducer runner_producer)
{
  m_producers[execution_strategy] = runner_producer;
}

std::vector<std::string> RunnerFactory::supported_execution_strategies() const
{
  std::vector<std::string> keys;
  for (const auto & kvp : m_producers) {
    keys.push_back(kvp.first);
  }
  return keys;
}

std::unique_ptr<Runner> RunnerFactory::create_runner(const ExperimentConfiguration & ec) const
{
  return m_producers.at(ec.execution_strategy)(ec);
}

}  // namespace performance_test
