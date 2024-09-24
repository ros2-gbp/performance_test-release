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

#ifndef PERFORMANCE_TEST__EXPERIMENT_METRICS__ANALYSIS_RESULT_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_METRICS__ANALYSIS_RESULT_HPP_

#include <chrono>
#include <sstream>
#include <string>

#if !defined(WIN32)
#include <sys/time.h>
#include <sys/resource.h>
#endif  // !defined(WIN32)

#if defined(QNX)
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <sys/types.h>
#endif  // defined(QNX)

#include "performance_test/utilities/cpu_usage_tracker.hpp"
#include "performance_test/utilities/sample_statistics.hpp"

namespace performance_test
{

#if !defined(WIN32)
std::ostream & operator<<(std::ostream & stream, const timeval & e);
#endif  // !defined(WIN32)

struct AnalysisResult
{
  AnalysisResult();
  static std::string csv_header(const bool pretty_print = false, std::string st = ",");
  std::string to_csv_string(const bool pretty_print = false, std::string st = ",") const;
  size_t latency_seconds_n() const;
  double latency_seconds_min() const;
  double latency_seconds_max() const;
  double latency_seconds_mean() const;
  double latency_seconds_m2() const;
  double latency_seconds_variance() const;

  std::chrono::nanoseconds m_experiment_start = {};
  std::chrono::nanoseconds m_time_between_two_measurements = {};
  uint64_t m_num_samples_received = {};
  uint64_t m_num_samples_sent = {};
  uint64_t m_num_samples_lost = {};
  std::size_t m_total_data_received = {};

  SampleStatistics<std::int64_t> m_latency_stats;
#if !defined(WIN32)
  rusage m_sys_usage;
#endif  // !defined(WIN32)
  CpuInfo m_cpu_info;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_METRICS__ANALYSIS_RESULT_HPP_
