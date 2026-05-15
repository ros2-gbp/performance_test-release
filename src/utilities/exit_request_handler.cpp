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

#include "performance_test/utilities/exit_request_handler.hpp"

#include <csignal>

static void handle_sigint(int sig)
{
  std::signal(sig, SIG_DFL);
  performance_test::ExitRequestHandler::get().request_exit();
}

namespace performance_test
{

ExitRequestHandler::ExitRequestHandler()
: m_exit_requested(false) {}

void ExitRequestHandler::setup()
{
  std::signal(SIGINT, handle_sigint);
}

bool ExitRequestHandler::exit_requested() const
{
  return m_exit_requested;
}

void ExitRequestHandler::request_exit()
{
  m_exit_requested = true;
}

}  // namespace performance_test
