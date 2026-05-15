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

#include "performance_test/outputs/output_factory.hpp"

#include <iostream>
#include <memory>
#include <regex>
#include <vector>

#include "performance_test/outputs/csv_output.hpp"
#include "performance_test/outputs/stdout_output.hpp"
#include "performance_test/outputs/json_output.hpp"

namespace performance_test
{
std::vector<std::shared_ptr<Output>> OutputFactory::get(const OutputConfiguration & config)
{
  std::vector<std::shared_ptr<Output>> outputs;

  if (config.print_to_console) {
    std::cout << "WARNING: Printing to the console degrades the performance." << std::endl;
    std::cout << "It is recommended to use a log file instead with --logfile." << std::endl;
    outputs.push_back(std::make_shared<StdoutOutput>());
  }

  if (!config.logfile_path.empty()) {
    if (std::regex_match(config.logfile_path, std::regex(".*\\.csv$"))) {
      outputs.push_back(std::make_shared<CsvOutput>(config.logfile_path));
    } else if (std::regex_match(config.logfile_path, std::regex(".*\\.json$"))) {
      outputs.push_back(std::make_shared<JsonOutput>(config.logfile_path));
    } else {
      std::cerr << "Unsupported log file type: " << config.logfile_path << std::endl;
      std::terminate();
    }
  }

  if (outputs.empty()) {
    std::cout << "WARNING: No output configured" << std::endl;
  }

  return outputs;
}
}  // namespace performance_test
