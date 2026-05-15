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

#ifndef PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_REGISTRY_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_REGISTRY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{
class PubSubRegistry
{
public:
  virtual ~PubSubRegistry() = default;

  typedef std::string Communicator;

  typedef std::string Message;

  typedef std::function<std::unique_ptr<Publisher>(
        const ExperimentConfiguration &)> PublisherProducer;

  typedef std::function<std::unique_ptr<Subscriber>(
        const ExperimentConfiguration &)> SubscriberProducer;

  virtual void register_pub_sub(
    const Communicator & communicator,
    const Message & message,
    PublisherProducer publisher_producer,
    SubscriberProducer subscriber_producer) = 0;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_REGISTRY_HPP_
