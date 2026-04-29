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

#ifndef PERFORMANCE_TEST__PLUGIN__SUBSCRIBER_HPP_
#define PERFORMANCE_TEST__PLUGIN__SUBSCRIBER_HPP_

#include <stdexcept>

#include "performance_test/experiment_metrics/message_received_listener.hpp"

namespace performance_test
{

class Subscriber
{
public:
  virtual ~Subscriber() = default;

  /// @brief Prepare for communication
  /// This is called once, after all Publishers and Subscribers
  /// are created, and before any messages are sent.
  /// This is a good place to put blocking operations that do not belong in
  /// the constructor, such as participant discovery.
  virtual void prepare() {}

  virtual void update_subscription(MessageReceivedListener & listener) = 0;

  virtual void take(MessageReceivedListener &)
  {
    throw std::runtime_error("This subscriber does not support take!");
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__SUBSCRIBER_HPP_
