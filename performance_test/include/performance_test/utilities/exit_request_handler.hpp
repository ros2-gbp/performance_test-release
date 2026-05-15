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

#ifndef PERFORMANCE_TEST__UTILITIES__EXIT_REQUEST_HANDLER_HPP_
#define PERFORMANCE_TEST__UTILITIES__EXIT_REQUEST_HANDLER_HPP_

#include <atomic>

namespace performance_test
{
class ExitRequestHandler
{
public:
  static ExitRequestHandler & get()
  {
    static ExitRequestHandler instance;

    return instance;
  }

  ExitRequestHandler(ExitRequestHandler const &) = delete;
  ExitRequestHandler(ExitRequestHandler &&) = delete;

  ExitRequestHandler & operator=(ExitRequestHandler const &) = delete;
  ExitRequestHandler & operator=(ExitRequestHandler &&) = delete;

  void setup();
  bool exit_requested() const;
  void request_exit();

private:
  ExitRequestHandler();

  std::atomic_bool m_exit_requested;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILITIES__EXIT_REQUEST_HANDLER_HPP_
