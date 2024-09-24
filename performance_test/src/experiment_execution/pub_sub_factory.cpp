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

#include "performance_test/experiment_execution/pub_sub_factory.hpp"

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace performance_test
{
void PubSubFactory::register_pub_sub(
  const Communicator & communicator,
  const Message & message,
  PublisherProducer publisher_producer,
  SubscriberProducer subscriber_producer)
{
  m_pub_producers[communicator][message] = publisher_producer;
  m_sub_producers[communicator][message] = subscriber_producer;
}

std::vector<std::string> PubSubFactory::supported_communicators() const
{
  std::set<std::string> keys;
  for (const auto & kvp : m_pub_producers) {
    keys.insert(kvp.first);
  }
  return std::vector<std::string>(keys.begin(), keys.end());
}

std::vector<std::string> PubSubFactory::supported_messages() const
{
  std::set<std::string> keys;
  for (const auto & kvp : m_pub_producers) {
    for (const auto & kvp2 : kvp.second) {
      keys.insert(kvp2.first);
    }
  }
  return std::vector<std::string>(keys.begin(), keys.end());
}

std::unique_ptr<Publisher> PubSubFactory::create_publisher(
  const ExperimentConfiguration & ec) const
{
  return m_pub_producers.at(ec.communicator).at(ec.msg_name)(ec);
}

std::unique_ptr<Subscriber> PubSubFactory::create_subscriber(
  const ExperimentConfiguration & ec) const
{
  return m_sub_producers.at(ec.communicator).at(ec.msg_name)(ec);
}

}  // namespace performance_test
