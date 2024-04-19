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

#ifndef PERFORMANCE_TEST__PLUGIN__PUBLISHER_HPP_
#define PERFORMANCE_TEST__PLUGIN__PUBLISHER_HPP_

#include <cstdint>

#include "performance_test/utilities/msg_traits.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{

class Publisher
{
public:
  virtual ~Publisher() = default;

  virtual void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

  virtual void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

protected:
  template<typename T>
  inline void init_msg(
    T & msg,
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id)
  {
    MsgTraits::ensure_fixed_size(msg);
    msg.id = sample_id;
    msg.time = timestamp_provider.get();
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__PUBLISHER_HPP_
