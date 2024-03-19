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

#include "performance_test/experiment_execution/round_trip_relay_runner.hpp"

#include <exception>
#include <memory>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_factory.hpp"
#include "performance_test/experiment_execution/runner.hpp"

namespace performance_test
{
RoundTripRelayRunner::RoundTripRelayRunner(const ExperimentConfiguration & ec)
: Runner(ec),
  m_relay(
    ec,
    PubSubFactory::get().create_publisher(ec),
    PubSubFactory::get().create_subscriber(ec))
{
  if (ec.number_of_publishers != 1) {
    throw std::invalid_argument(
            "Round-trip relay requires exactly one publisher.");
  }
  if (ec.number_of_subscribers != 1) {
    throw std::invalid_argument(
            "Round-trip relay requires exactly one subscriber.");
  }
  if (ec.is_zero_copy_transfer) {
    throw std::invalid_argument(
            "Round-trip relay can not use loaned messages (zero copy).");
  }
}

RoundTripRelayRunner::~RoundTripRelayRunner()
{
  m_thread->join();
}

void RoundTripRelayRunner::run_pubs_and_subs()
{
  m_thread = std::make_unique<std::thread>(
    [this]() {
      while (m_running) {
        m_relay.run();
      }
    }
  );
}

}  // namespace performance_test
