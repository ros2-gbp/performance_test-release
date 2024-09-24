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

#ifndef PERFORMANCE_TEST__PLUGINS__DEMO__SUBSCRIBER_IMPL_HPP_
#define PERFORMANCE_TEST__PLUGINS__DEMO__SUBSCRIBER_IMPL_HPP_

#include "performance_test/plugin/subscriber.hpp"

#include <condition_variable>
#include <mutex>
#include <poll.h>
#include <sys/socket.h>
#include <thread>

namespace performance_test
{
// Each plugin should provide a Subscriber implementation.
// The Subscriber is used by the perf_test application to receive messages.
// The Subscriber will generally wrap a middleware's data reader or subscriber.
// This (crude) implentation wraps a simple Unix socket.
template<class DataType>
class SubscriberImpl : public Subscriber
{
public:
  explicit SubscriberImpl(const ExperimentConfiguration & ec)
  : accepter([this](){
      std::unique_lock lock(m_mutex);
      start_accepter.wait(lock);
      connection_fd = accept(sfd, nullptr, nullptr);
    })
  {
    // Create and configure the data reader, QoS settings, etc

    sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    int option = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    uint16_t port = 8964;

    struct sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(port);

    int bind_ret = bind(sfd,
                        reinterpret_cast<struct sockaddr *>(&saddr),
                        sizeof(saddr));
    if (bind_ret < 0) {
      throw std::runtime_error("Failed to socket to address");
    }

    int listen_ret = listen(sfd, 1);
    if (listen_ret < 0) {
      throw std::runtime_error("Failed to listen for connections");
    }

    start_accepter.notify_all();
  }

  ~SubscriberImpl()
  {
    accepter.join();
    close(sfd);
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    // Prepare to receive a message, and then read it.
    // This usually uses a waitset or polls the data reader.
    struct pollfd pfds[1];
    pfds[0].fd = connection_fd;
    pfds[0].events = POLLIN;
    poll(pfds, 1, 1000);
    if (pfds[0].revents & POLLIN) {
      this->take(listener);
    }
  }

  void take(MessageReceivedListener & listener) override
  {
    // Take a message from the data reader. This can be called from
    // update_subscription(), after the waitset or poll indicates that a
    // message is ready.
    // In some cases, such as the intra-thread execution strategy,
    // it is known that a message has already been written by the Publisher,
    // and it is safe to directly take the message without being notified
    // by a waitset or similar triggering mechanism.

    // Capture the timestamp as soon as a message is available, so the latency
    // measurement is as accurate as possible.
    const auto received_time = now_int64_t();

    DataType data;
    recv(connection_fd, &data, sizeof(data), 0);

    // The message metrics will be processed and logged in the core of perf_test.
    // The Plugin only needs to notify the MessageReceivedListener for each message.
    listener.on_message_received(
      data.time,
      received_time,
      data.id,
      sizeof(DataType)
    );
  }

private:
  int sfd;
  int connection_fd;
  uint8_t buffer[sizeof(DataType)];
  std::thread accepter;
  std::condition_variable start_accepter;
  std::mutex m_mutex;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__DEMO__SUBSCRIBER_IMPL_HPP_
