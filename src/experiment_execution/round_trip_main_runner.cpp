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

#include "performance_test/experiment_execution/round_trip_main_runner.hpp"

#include <exception>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/inter_thread_runner.hpp"

namespace performance_test
{
RoundTripMainRunner::RoundTripMainRunner(const ExperimentConfiguration & ec)
: InterThreadRunner(ec)
{
  if (ec.number_of_publishers != 1) {
    throw std::invalid_argument(
            "Round-trip main requires exactly one publisher.");
  }
  if (ec.number_of_subscribers != 1) {
    throw std::invalid_argument(
            "Round-trip main requires exactly one subscriber.");
  }
  if (ec.is_zero_copy_transfer) {
    throw std::invalid_argument(
            "Round-trip main can not use loaned messages (zero copy).");
  }
}

}  // namespace performance_test
