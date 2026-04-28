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

#ifndef PERFORMANCE_TEST__UTILITIES__RT_ENABLER_HPP_
#define PERFORMANCE_TEST__UTILITIES__RT_ENABLER_HPP_

#include <cstdint>

namespace performance_test
{
///
/// CPU affinity related pre initialization to increase determinism and
/// thereby reduce latency for the entire process
/// \brief Perform some RT related process initialization
/// \param[in] cpu_bit_mask Default cpu affinity of all the threads in the process
/// \param[in] prio default prio of all the threads in the process
/// \return 0 on success, throw an exception on error
void pre_proc_rt_init(const uint32_t cpu_bit_mask, const int32_t prio);

///
/// Mem related post initialization to increase determinism and
/// thereby reduce latency for the entire process
/// \brief Perform some RT related memory process initialization
/// \return 0 on success, throw an exception on error
void post_proc_rt_init();
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILITIES__RT_ENABLER_HPP_
