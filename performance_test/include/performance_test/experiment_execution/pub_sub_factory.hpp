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

#ifndef PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_FACTORY_HPP_
#define PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_FACTORY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_execution/pub_sub_registry.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{
class PubSubFactory : public PubSubRegistry
{
public:
  static PubSubFactory & get()
  {
    static PubSubFactory instance;

    return instance;
  }

  PubSubFactory(PubSubFactory const &) = delete;
  PubSubFactory(PubSubFactory &&) = delete;

  PubSubFactory & operator=(PubSubFactory const &) = delete;
  PubSubFactory & operator=(PubSubFactory &&) = delete;

  void register_pub_sub(
    const Communicator & communicator,
    const Message & message,
    PublisherProducer publisher_producer,
    SubscriberProducer subscriber_producer) override;
  std::vector<std::string> supported_communicators() const;
  std::vector<std::string> supported_messages() const;
  std::unique_ptr<Publisher> create_publisher(const ExperimentConfiguration & ec) const;
  std::unique_ptr<Subscriber> create_subscriber(const ExperimentConfiguration & ec) const;

private:
  PubSubFactory() = default;

  std::map<Communicator, std::map<Message, PublisherProducer>> m_pub_producers;
  std::map<Communicator, std::map<Message, SubscriberProducer>> m_sub_producers;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXPERIMENT_EXECUTION__PUB_SUB_FACTORY_HPP_
