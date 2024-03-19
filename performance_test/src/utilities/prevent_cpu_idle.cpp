// Copyright 2023-2024 Apex.AI, Inc.
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

#include "performance_test/utilities/prevent_cpu_idle.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <iostream>

#if defined(QNX)
#include <sys/syspage.h>
#include <sys/procmgr.h>
#include <sys/sysmgr.h>
#endif

namespace performance_test
{
void prevent_cpu_idle()
{
#if defined(__linux__)
  static std::unique_ptr<FILE, int (*)(FILE *)> cpu_dma_latency
  {
    ::fopen("/dev/cpu_dma_latency", "wb"),
    ::fclose
  };

  if (!cpu_dma_latency) {
    if (errno == ENOENT) {
      throw std::runtime_error("The file /dev/cpu_dma_latency does not exist");
    }
    throw std::runtime_error("Failed to open /dev/cpu_dma_latency");
  }

  ::setbuf(cpu_dma_latency.get(), nullptr);

  std::int32_t latency_i32 = 0;
  if (::fwrite(&latency_i32, sizeof(latency_i32), 1, cpu_dma_latency.get()) != 1) {
    throw std::runtime_error("Failed to write latency specification to /dev/cpu_dma_latency");
  }
#elif defined(__QNX__)
  int e1 = procmgr_ability(
    0,
    PROCMGR_ADN_ROOT | PROCMGR_ADN_NONROOT | PROCMGR_AID_RUNSTATE_BURST | PROCMGR_AOP_ALLOW,
    PROCMGR_AID_EOL);
  if (e1 != EOK) {
    throw std::runtime_error("Failed to enable PROCMGR_AID_RUNSTATE_BURST");
  }
  for (std::uint32_t i = 0; i < _syspage_ptr->num_cpu; i++) {
    int e2 = sysmgr_runstate(i, 1);
    if (e2 != EOK) {
      throw std::runtime_error("Failed to set the runstate");
    }
    int e3 = sysmgr_runstate_dynamic(i, 0);
    if (e3 != EOK) {
      throw std::runtime_error("Failed to set the dynamic runstate");
    }
  }
#else
  throw std::runtime_error("prevent_cpu_idle is not supported on this platform");
#endif
}
}  // namespace performance_test
