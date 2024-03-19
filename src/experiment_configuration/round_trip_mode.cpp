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

#include "performance_test/experiment_configuration/round_trip_mode.hpp"

#include <string>

namespace performance_test
{

std::string to_string(const RoundTripMode rtm)
{
  if (rtm == RoundTripMode::NONE) {
    return "None";
  }
  if (rtm == RoundTripMode::MAIN) {
    return "Main";
  }
  if (rtm == RoundTripMode::RELAY) {
    return "Relay";
  }
  throw std::invalid_argument("Enum value not supported!");
}

std::ostream & operator<<(std::ostream & stream, const RoundTripMode rtm)
{
  return stream << to_string(rtm);
}

RoundTripMode round_trip_mode_from_string(const std::string & s)
{
  if (s == "None") {
    return RoundTripMode::NONE;
  }
  if (s == "Main") {
    return RoundTripMode::MAIN;
  }
  if (s == "Relay") {
    return RoundTripMode::RELAY;
  }
  throw std::invalid_argument("Invalid round trip mode string!");
}

}  // namespace performance_test
