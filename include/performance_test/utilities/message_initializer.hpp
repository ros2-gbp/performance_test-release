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

#ifndef PERFORMANCE_TEST__UTILITIES__MESSAGE_INITIALIZER_HPP_
#define PERFORMANCE_TEST__UTILITIES__MESSAGE_INITIALIZER_HPP_

#include <cstdint>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/utilities/msg_traits.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{

class MessageInitializer
{
public:
  explicit MessageInitializer(const ExperimentConfiguration & ec)
  : m_ec(ec) {}

  template<typename T>
  inline void init_msg(
    T & msg,
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id)
  {
    init_bounded_sequence(msg);
    init_unbounded_sequence(msg);
    init_unbounded_string(msg);
    init_sample_id(msg, sample_id);
    init_timestamp(msg, timestamp_provider);
  }

private:
  const ExperimentConfiguration & m_ec;

  template<typename T>
  inline
  void init_bounded_sequence(T & msg)
  {
    if constexpr (MsgTraits::has_bounded_sequence<T>::value) {
      msg.bounded_sequence.resize(msg.bounded_sequence.capacity());
    } else if constexpr (MsgTraits::has_bounded_sequence_func<T>::value) {
      msg.bounded_sequence().resize(msg.bounded_sequence().capacity());
    }
  }

  template<typename T>
  inline
  void init_unbounded_sequence(T & msg)
  {
    if constexpr (MsgTraits::has_unbounded_sequence<T>::value) {
      msg.unbounded_sequence.resize(m_ec.unbounded_msg_size);
    } else if constexpr (MsgTraits::has_unbounded_sequence_func<T>::value) {
      msg.unbounded_sequence().resize(m_ec.unbounded_msg_size);
    }
  }

  template<typename T>
  inline
  void init_unbounded_string(T & msg)
  {
    if constexpr (MsgTraits::has_unbounded_string<T>::value) {
      msg.unbounded_string.resize(m_ec.unbounded_msg_size);
    } else if constexpr (MsgTraits::has_unbounded_string_func<T>::value) {
      msg.unbounded_string().resize(m_ec.unbounded_msg_size);
    }
  }

  template<typename T>
  inline void init_sample_id(
    T & msg,
    std::uint64_t sample_id)
  {
    if constexpr (MsgTraits::has_id_object<T>::value) {
      msg.id = sample_id;
    } else if constexpr (MsgTraits::has_id_function<T>::value) {
      msg.id(sample_id);
    }
  }

  template<typename T>
  inline void init_timestamp(
    T & msg,
    const TimestampProvider & timestamp_provider)
  {
    if constexpr (MsgTraits::has_time_object<T>::value) {
      msg.time = timestamp_provider.get();
    } else if constexpr (MsgTraits::has_time_function<T>::value) {
      msg.time(timestamp_provider.get());
    }
  }
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILITIES__MESSAGE_INITIALIZER_HPP_
