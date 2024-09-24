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

#ifndef PERFORMANCE_TEST__EXPERIMENT_METRICS__MESSAGE_RECEIVED_LISTENER_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_METRICS__MESSAGE_RECEIVED_LISTENER_HPP_

#include <cstdint>

namespace performance_test
{
class MessageReceivedListener
{
public:
  virtual ~MessageReceivedListener() = default;

  virtual void on_message_received(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t time_msg_received_ns,
    const std::uint64_t sample_id,
    const std::size_t data_type_size
  ) = 0;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_METRICS__MESSAGE_RECEIVED_LISTENER_HPP_
