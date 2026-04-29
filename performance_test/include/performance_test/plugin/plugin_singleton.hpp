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

#ifndef PERFORMANCE_TEST__PLUGIN__PLUGIN_SINGLETON_HPP_
#define PERFORMANCE_TEST__PLUGIN__PLUGIN_SINGLETON_HPP_

#include <memory>

#include "performance_test/plugin/plugin.hpp"
#include "performance_test/plugin/plugin_factory.hpp"

namespace performance_test
{
class PluginSingleton
{
public:
  static std::shared_ptr<Plugin> & get()
  {
    static PluginSingleton instance;

    return instance.m_plugin;
  }

  PluginSingleton(PluginSingleton const &) = delete;
  PluginSingleton(PluginSingleton &&) = delete;

  PluginSingleton & operator=(PluginSingleton const &) = delete;
  PluginSingleton & operator=(PluginSingleton &&) = delete;

private:
  PluginSingleton()
  : m_plugin(PluginFactory::create_plugin()) {}

  std::shared_ptr<Plugin> m_plugin;
};
}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGIN__PLUGIN_SINGLETON_HPP_
