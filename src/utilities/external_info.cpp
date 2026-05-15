// Copyright 2019-2024 Apex.AI, Inc.
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

#include "performance_test/utilities/external_info.hpp"

#include <map>
#include <string>

#include <rapidjson/document.h>

namespace performance_test
{
std::string ExternalInfo::as_string()
{
  std::string accum;
  for (auto & p : as_map()) {
    accum += p.first + ": " + p.second + "\n";
  }
  return accum;
}

std::map<std::string, std::string> ExternalInfo::as_map()
{
  std::map<std::string, std::string> accum;
  const auto ptr = std::getenv("APEX_PERFORMANCE_TEST");
  if (ptr) {
    rapidjson::Document document;
    document.Parse(ptr);

    for (auto & m : document.GetObject()) {
      accum[m.name.GetString()] = m.value.GetString();
    }
  }
  return accum;
}
}  // namespace performance_test
