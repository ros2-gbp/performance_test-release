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

#ifndef PERFORMANCE_TEST__UTILITIES__TIMESTAMP_PROVIDER_HPP_
#define PERFORMANCE_TEST__UTILITIES__TIMESTAMP_PROVIDER_HPP_

#include "performance_test/utilities/perf_clock.hpp"

namespace performance_test
{
class TimestampProvider
{
public:
  virtual ~TimestampProvider() = default;

  virtual std::int64_t get() const = 0;
};

class PublisherTimestampProvider : public TimestampProvider
{
public:
  std::int64_t get() const override
  {
    return now_int64_t();
  }
};

class RoundtripTimestampProvider : public TimestampProvider
{
public:
  explicit RoundtripTimestampProvider(std::int64_t timestamp)
  : m_timestamp(timestamp) {}

  std::int64_t get() const override
  {
    return m_timestamp;
  }

private:
  const std::int64_t m_timestamp;
};
}  // namespace performance_test
#endif  // PERFORMANCE_TEST__UTILITIES__TIMESTAMP_PROVIDER_HPP_
