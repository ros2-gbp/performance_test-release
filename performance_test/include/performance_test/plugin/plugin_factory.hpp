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

#ifndef PERFORMANCE_TEST__PLUGIN__PLUGIN_FACTORY_HPP_
#define PERFORMANCE_TEST__PLUGIN__PLUGIN_FACTORY_HPP_

#include "performance_test/plugin/plugin.hpp"

#include <memory>

namespace performance_test
{
class PluginFactory
{
public:
  // The plugin implementation must define this function
  static std::shared_ptr<Plugin> create_plugin();
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__PLUGIN_FACTORY_HPP_
