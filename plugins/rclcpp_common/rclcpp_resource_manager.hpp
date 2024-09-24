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

#ifndef PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_RESOURCE_MANAGER_HPP_
#define PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_RESOURCE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{
class RclcppResourceManager
{
public:
  static RclcppResourceManager & get()
  {
    static RclcppResourceManager instance;
    return instance;
  }

  RclcppResourceManager(RclcppResourceManager const &) = delete;
  RclcppResourceManager(RclcppResourceManager &&) = delete;
  RclcppResourceManager & operator=(RclcppResourceManager const &) = delete;
  RclcppResourceManager & operator=(RclcppResourceManager &&) = delete;

  std::shared_ptr<rclcpp::Node> rclcpp_node(const ExperimentConfiguration & ec) const
  {
    /* Temporarely commented out until ROS2 waitsets are available. As of now every
    * ROS2 thread needs a node in the current architecture.
    */

    // std::lock_guard<std::mutex> lock(m_global_mutex);
    // if(!m_node)
    // {
    //   m_node = rclcpp::Node::make_shared("performance_test", "", ec.use_ros_shm());
    // }
    // return m_node;

    std::string rand_str;
    // if security is enabled, use a fixed id
    if (ec.with_security) {
      static uint32_t id = 0;
      rand_str = std::to_string(id++);
    } else {
      rand_str = std::to_string(std::rand());
    }

    auto options = rclcpp::NodeOptions();

    auto env_name = "ROS_DOMAIN_ID";
    auto env_value = std::to_string(ec.dds_domain_id);
#ifdef _WIN32
    _putenv_s(env_name, env_value.c_str());
#else
    setenv(env_name, env_value.c_str(), true);
#endif

    return rclcpp::Node::make_shared("performance_test" + rand_str, options);
  }

private:
  RclcppResourceManager()
  : m_node(nullptr) {}

  mutable std::shared_ptr<rclcpp::Node> m_node;
  mutable std::mutex m_global_mutex;
};

}  // namespace performance_test
#endif  // PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_RESOURCE_MANAGER_HPP_
