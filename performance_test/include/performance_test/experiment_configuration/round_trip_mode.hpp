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

#ifndef PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__ROUND_TRIP_MODE_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__ROUND_TRIP_MODE_HPP_

#include <ostream>
#include <string>
#include <vector>

namespace performance_test
{
enum class RoundTripMode
{
  NONE,  /// No roundtrip. Samples are only sent from sender to reciever.
  MAIN,  /// Sends packages to the relay and receives packages from the relay.
  RELAY  /// Relays packages from MAIN back to MAIN.
};

std::string to_string(const RoundTripMode rtm);

std::ostream & operator<<(std::ostream & stream, const RoundTripMode rtm);

RoundTripMode round_trip_mode_from_string(const std::string & s);

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__ROUND_TRIP_MODE_HPP_
