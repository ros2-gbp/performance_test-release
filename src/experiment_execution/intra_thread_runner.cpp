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

#include "performance_test/experiment_execution/intra_thread_runner.hpp"

#include <exception>
#include <memory>
#include <thread>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/data_entity_runner.hpp"

namespace performance_test
{
IntraThreadRunner::IntraThreadRunner(const ExperimentConfiguration & ec)
: DataEntityRunner(ec)
{
  if (ec.number_of_publishers != 1) {
    throw std::invalid_argument(
            "Intra-thread execution requires exactly one publisher.");
  }
  if (ec.number_of_subscribers < 1) {
    throw std::invalid_argument(
            "Intra-thread execution requires at least one subscriber.");
  }
  if (!ec.use_loaned_samples) {
    throw std::invalid_argument(
            "Intra-thread execution only works with loaned messages (zero copy).");
  }
  if (ec.roundtrip_mode != RoundTripMode::NONE) {
    throw std::invalid_argument(
            "Intra-thread execution only works with RoundTripMode NONE.");
  }
}

IntraThreadRunner::~IntraThreadRunner()
{
  m_thread->join();
}

void IntraThreadRunner::run_pubs_and_subs()
{
  m_thread = std::make_unique<std::thread>(
    [this]() {
      for (auto & pub : m_pubs) {
        pub->prepare();
      }
      for (auto & sub : m_subs) {
        sub->prepare();
      }
      while (m_running) {
        for (auto & pub : m_pubs) {
          pub->run();
        }
        for (auto & sub : m_subs) {
          sub->take();
        }
      }
    }
  );
}

}  // namespace performance_test
