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

#include "performance_test/outputs/csv_output.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "performance_test/experiment_metrics/analysis_result.hpp"
#include "performance_test/utilities/external_info.hpp"

namespace performance_test
{
CsvOutput::CsvOutput(const std::string & logfile_path)
: m_logfile_path(logfile_path) {}

CsvOutput::~CsvOutput()
{
  close();
}

void CsvOutput::open(const ExperimentConfiguration & ec)
{
  if (!m_logfile_path.empty()) {
    m_os.open(m_logfile_path, std::ofstream::out);
    m_is_open = true;

    std::cout << "Writing CSV output to: " << m_logfile_path << std::endl;

    // write experiment details
    m_os << ec;
    m_os << ExternalInfo::as_string();
    m_os << std::endl << std::endl;

    // write header
    m_os << "---EXPERIMENT-START---" << std::endl;
    m_os << AnalysisResult::csv_header(true) << std::endl;
  }
}
void CsvOutput::update(const AnalysisResult & result)
{
  m_os << result.to_csv_string(true) << std::endl;
}

void CsvOutput::close()
{
  if (m_is_open) {
    m_os.close();
  }
}

}  // namespace performance_test
