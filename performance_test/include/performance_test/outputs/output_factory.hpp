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

#ifndef PERFORMANCE_TEST__OUTPUTS__OUTPUT_FACTORY_HPP_
#define PERFORMANCE_TEST__OUTPUTS__OUTPUT_FACTORY_HPP_

#include <memory>
#include <vector>

#include "performance_test/experiment_configuration/output_configuration.hpp"
#include "performance_test/outputs/output.hpp"

namespace performance_test
{
class OutputFactory
{
public:
  static std::vector<std::shared_ptr<Output>> get(const OutputConfiguration & config);
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__OUTPUTS__OUTPUT_FACTORY_HPP_
