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

#ifndef PERFORMANCE_TEST__UTILITIES__PERF_CLOCK_HPP_
#define PERFORMANCE_TEST__UTILITIES__PERF_CLOCK_HPP_

#include <chrono>

#if defined(QNX)
#include <inttypes.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#endif  // defined(QNX)

namespace performance_test
{
using perf_clock = std::chrono::steady_clock;

inline std::int64_t now_int64_t()
{
#if defined(QNX)
  return static_cast<std::int64_t>(ClockCycles());
#else
  return perf_clock::now().time_since_epoch().count();
#endif  // defined(QNX)
}
}  // namespace performance_test
#endif  // PERFORMANCE_TEST__UTILITIES__PERF_CLOCK_HPP_
