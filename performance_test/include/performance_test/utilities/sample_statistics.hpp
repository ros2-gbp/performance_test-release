// Copyright 2023-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__UTILITIES__SAMPLE_STATISTICS_HPP_
#define PERFORMANCE_TEST__UTILITIES__SAMPLE_STATISTICS_HPP_

#include <limits>
#include <algorithm>

namespace performance_test
{
template<typename TNum>
class SampleStatistics
{
public:
  SampleStatistics()
  {
    clear();
    static_assert(std::numeric_limits<double>::is_iec559, "Non IEEE754 are not supported.");
  }

  void clear()
  {
    m_n = 0;
    m_sum = 0;
    m_sum_of_sq = 0;
    m_min = std::numeric_limits<TNum>::max();
    m_max = std::numeric_limits<TNum>::lowest();
  }

  void add_sample(const TNum x)
  {
    m_n += 1;
    m_sum += x;
    m_sum_of_sq += x * x;
    m_min = std::min(m_min, x);
    m_max = std::max(m_max, x);
  }

  void combine(const SampleStatistics & other)
  {
    m_n += other.m_n;
    m_sum += other.m_sum;
    m_sum_of_sq += other.m_sum_of_sq;
    m_min = std::min(m_min, other.m_min);
    m_max = std::max(m_max, other.m_max);
  }

  std::size_t n() const
  {
    return m_n;
  }

  double mean() const
  {
    return static_cast<double>(m_sum) / static_cast<double>(m_n);
  }

  /// The second moment
  double m2() const
  {
    double sum_of_sq = static_cast<double>(m_sum_of_sq);
    double sq_of_sum = static_cast<double>(m_sum * m_sum);
    return sum_of_sq - sq_of_sum / static_cast<double>(m_n);
  }

  double variance() const
  {
    return m2() / static_cast<double>(m_n - 1);
  }

  TNum min() const
  {
    return m_min;
  }

  TNum max() const
  {
    return m_max;
  }

private:
  std::size_t m_n;
  TNum m_sum;
  TNum m_sum_of_sq;
  TNum m_min;
  TNum m_max;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILITIES__SAMPLE_STATISTICS_HPP_
