// Copyright 2021-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__OUTPUTS__JSON_OUTPUT_HPP_
#define PERFORMANCE_TEST__OUTPUTS__JSON_OUTPUT_HPP_

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <fstream>
#include <string>

#include "performance_test/experiment_configuration/experiment_configuration.hpp"
#include "performance_test/experiment_metrics/analysis_result.hpp"
#include "performance_test/outputs/output.hpp"

namespace performance_test
{

class JsonOutput : public Output
{
public:
  explicit JsonOutput(const std::string & logfile_path);
  virtual ~JsonOutput();

  void open(const ExperimentConfiguration & ec) override;
  void update(const AnalysisResult & result) override;
  void close() override;

private:
  const std::string m_logfile_path;
  mutable std::ofstream m_os;
  rapidjson::StringBuffer m_sb;
  rapidjson::Writer<rapidjson::StringBuffer> m_writer;

  void write(const ExperimentConfiguration & ec);
  void write(const AnalysisResult & ar);
  void write(const char * key, const std::string & val);
  void write(const char * key, uint32_t val);
  void write(const char * key, uint64_t val);
  void write(const char * key, int32_t val);
  void write(const char * key, int64_t val);
  void write(const char * key, float val);
  void write(const char * key, double val);
  void write(const char * key, bool val);
  void write(const char * key, timeval val);
  void write(const char * key, const std::chrono::nanoseconds val);
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__OUTPUTS__JSON_OUTPUT_HPP_
