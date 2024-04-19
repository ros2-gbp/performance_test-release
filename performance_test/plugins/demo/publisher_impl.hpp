// Copyright 2021-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__DEMO__PUBLISHER_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__DEMO__PUBLISHER_IMPL_HPP_

#include "performance_test/plugin/publisher.hpp"

#include <netdb.h>
#include <sys/socket.h>

namespace performance_test
{
// Each plugin should provide a Publisher implementation.
// The Publisher is used by the perf_test application to send messages.
// The Publisher will generally wrap a middleware's data writer or publisher.
// This (crude) implentation wraps a simple Unix socket.
template<class DataType>
class PublisherImpl : public Publisher
{
public:
  explicit PublisherImpl(const ExperimentConfiguration & ec)
  {
    // Create and configure the data writer, QoS settings, etc

    sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    int option = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    struct hostent * host_entry = gethostbyname("localhost");
    if (!host_entry) {
      throw std::runtime_error("Failed to get host by name");
    }
    uint16_t port = 8964;

    struct sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = reinterpret_cast<struct in_addr *>(host_entry->h_addr_list[0])->s_addr;
    saddr.sin_port = htons(port);

    int connect_ret = connect(sfd,
                              reinterpret_cast<struct sockaddr *>(&saddr),
                              sizeof(saddr));
    if (connect_ret < 0) {
      throw std::runtime_error("Failed to connect");
    }
  }

  ~PublisherImpl()
  {
    close(sfd);
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    // This is called when zero-copy transfer is not enabled.

    // Invoke init_msg just before sending, so that the timestamp provided
    // by the TimestampProvider is as fresh as possible.
    // It is important to use the TimestampProvider instead of directly calling clock::now()
    // so that the Publisher can be used in Relay mode.
    DataType message;
    this->init_msg(message, timestamp_provider, sample_id);
    send(sfd, &message, sizeof(message), 0);
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    // This is called when zero-copy transfer is enabled.
    // If loaned samples are not supported by the middleware, then
    // raise an error here instead.
    throw std::runtime_error("Loaned samples are not supported by this plugin");

    // Invoke init_msg just before sending...
    // this->init_msg(loaned_sample, timestamp_provider, sample_id);
  }

private:
  int sfd;
  struct sockaddr_in socket_addr;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__DEMO__PUBLISHER_IMPL_HPP_
