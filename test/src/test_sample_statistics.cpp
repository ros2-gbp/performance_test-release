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

#include <cstdint>
#include <limits>
#include <cmath>

#include <gtest/gtest.h>

#include "performance_test/utilities/sample_statistics.hpp"

TEST(performance_test, SampleStatistics_init) {
  performance_test::SampleStatistics<std::int64_t> st;

  ASSERT_EQ(st.n(), 0);
  ASSERT_EQ(st.min(), std::numeric_limits<std::int64_t>::max());
  ASSERT_EQ(st.max(), std::numeric_limits<std::int64_t>::lowest());
  ASSERT_TRUE(std::isnan(st.mean()));  // divide by zero
  ASSERT_TRUE(std::isnan(st.variance()));  // divide by zero
}

TEST(performance_test, SampleStatistics_single_sample) {
  performance_test::SampleStatistics<std::int64_t> st;
  const std::int64_t sample = 5;
  st.add_sample(sample);

  ASSERT_EQ(st.n(), 1.0);
  ASSERT_EQ(st.min(), sample);
  ASSERT_EQ(st.max(), sample);
  ASSERT_DOUBLE_EQ(st.mean(), static_cast<double>(sample));
  ASSERT_TRUE(std::isnan(st.variance()));  // divide by zero
}


TEST(performance_test, SampleStatistics_two_samples) {
  performance_test::SampleStatistics<std::int64_t> st;

  const std::int64_t small = 5;
  const std::int64_t big = 50005;

  st.add_sample(small);
  st.add_sample(big);

  const double mean = (small + big) / 2.0;
  const double variance =
    ((mean - small) * (mean - small) +
    (mean - big) * (mean - big)) /
    (2.0 - 1.0);

  ASSERT_EQ(st.n(), 2);
  ASSERT_EQ(st.min(), small);
  ASSERT_EQ(st.max(), big);
  ASSERT_DOUBLE_EQ(st.mean(), mean);
  ASSERT_DOUBLE_EQ(st.variance(), variance);
}

TEST(performance_test, SampleStatistics_three_samples) {
  performance_test::SampleStatistics<std::int64_t> st;

  const std::int64_t small = 5;
  const std::int64_t mid = 1005;
  const std::int64_t big = 50005;

  st.add_sample(mid);
  st.add_sample(small);
  st.add_sample(big);

  const double mean = (small + big + mid) / 3.0;
  const double variance =
    ((mean - small) * (mean - small) +
    (mean - mid) * (mean - mid) +
    (mean - big) * (mean - big)) /
    (3.0 - 1.0);

  ASSERT_EQ(st.n(), 3);
  ASSERT_EQ(st.min(), small);
  ASSERT_EQ(st.max(), big);
  ASSERT_DOUBLE_EQ(st.mean(), mean);
  ASSERT_DOUBLE_EQ(st.variance(), variance);
}
