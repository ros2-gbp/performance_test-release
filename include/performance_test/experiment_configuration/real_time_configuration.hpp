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

#ifndef PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__REAL_TIME_CONFIGURATION_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__REAL_TIME_CONFIGURATION_HPP_

#include <ostream>
#include <string>

namespace performance_test
{
struct RealTimeConfiguration
{
  int32_t prio;
  uint32_t cpus;
  bool is_rt_init_required() const
  {
    return prio != 0 || cpus != 0;
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_CONFIGURATION__REAL_TIME_CONFIGURATION_HPP_
