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

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/utilities/message_initializer.hpp"
#include "performance_test/utilities/msg_traits.hpp"
#include "performance_test/utilities/timestamp_provider.hpp"

namespace performance_test
{

class Publisher
{
public:
  explicit Publisher(const ExperimentConfiguration & ec)
  : m_ec(ec), m_message_initializer(ec) {}

  virtual ~Publisher() = default;

  /// @brief Prepare for communication
  /// This is called once, after all Publishers and Subscribers
  /// are created, and before any messages are sent.
  /// This is a good place to put blocking operations that do not belong in
  /// the constructor, such as participant discovery.
  virtual void prepare() {}

  virtual void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

  virtual void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) = 0;

protected:
  const ExperimentConfiguration & m_ec;

  template<typename T>
  inline void init_msg(
    T & msg,
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id)
  {
    m_message_initializer.init_msg(msg, timestamp_provider, sample_id);
  }

private:
  MessageInitializer m_message_initializer;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__PUBLISHER_HPP_
