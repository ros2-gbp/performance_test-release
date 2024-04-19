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

#include "performance_test/cli/cli_parser.hpp"

#include <string>
#include <vector>

#include <tclap/CmdLine.h>

#include "performance_test/experiment_execution/pub_sub_factory.hpp"
#include "performance_test/experiment_execution/runner_factory.hpp"

namespace performance_test
{

CLIParser::CLIParser(int argc, char ** argv)
{
  try {
    TCLAP::CmdLine cmd("Apex.AI performance_test");

    TCLAP::SwitchArg printToConsoleArg("", "print-to-console",
      "Print metrics to console.", cmd, false);

    TCLAP::ValueArg<std::string> LogfileArg("l", "logfile",
      "Specify the name of the log file, e.g. -l \"log_$(date +%F_%H-%M-%S).json\"."
      " Supported formats: csv, json", false, "", "name", cmd);

    TCLAP::ValueArg<uint32_t> rateArg("r", "rate",
      "The publishing rate. 0 means publish as fast as possible. "
      "Default is 1000.", false, 1000, "N", cmd);

    std::vector<std::string> allowedCommunicators = PubSubFactory::get().supported_communicators();
    TCLAP::ValuesConstraint<std::string> allowedCommunicatorVals(allowedCommunicators);
    TCLAP::ValueArg<std::string> communicatorArg("c", "communicator",
      "The Plugin's communicator. "
      "Default is " + allowedCommunicators[0] + ".", false, allowedCommunicators[0],
      &allowedCommunicatorVals, cmd);

    std::vector<std::string> allowedExecStrats =
      RunnerFactory::get().supported_execution_strategies();
    TCLAP::ValuesConstraint<std::string> allowedExecStratVals(allowedExecStrats);
    TCLAP::ValueArg<std::string> executionStrategyArg("e", "execution-strategy",
      "The execution strategy to use. "
      "Default is INTER_THREAD.", false, "INTER_THREAD",
      &allowedExecStratVals, cmd);

    TCLAP::ValueArg<std::string> topicArg("t", "topic", "The topic name. Default is test_topic.",
      false, "test_topic", "topic", cmd);

    std::vector<std::string> allowedMsgs = PubSubFactory::get().supported_messages();
    TCLAP::ValuesConstraint<std::string> allowedMsgVals(allowedMsgs);
    TCLAP::ValueArg<std::string> msgArg("m", "msg", "The message type. "
      "Default is " + allowedMsgs[0] + ".", false, allowedMsgs[0], &allowedMsgVals, cmd);

    TCLAP::ValueArg<uint32_t> ddsDomainIdArg("", "dds-domain_id",
      "The DDS domain id. Default is 0.", false, 0, "id", cmd);

    std::vector<std::string> allowedReliabilityArgs{"RELIABLE", "BEST_EFFORT"};
    TCLAP::ValuesConstraint<std::string> allowedReliabilityArgsVals(allowedReliabilityArgs);
    TCLAP::ValueArg<std::string> reliabilityArg("", "reliability",
      "The QOS Reliability type. Default is BEST_EFFORT.", false, "BEST_EFFORT",
      &allowedReliabilityArgsVals, cmd);

    std::vector<std::string> allowedDurabilityArgs{"TRANSIENT_LOCAL", "VOLATILE"};
    TCLAP::ValuesConstraint<std::string> allowedDurabilityArgsVals(allowedDurabilityArgs);
    TCLAP::ValueArg<std::string> durabilityArg("", "durability",
      "The QOS Durability type. Default is VOLATILE.", false, "VOLATILE",
      &allowedDurabilityArgsVals, cmd);

    std::vector<std::string> allowedHistoryArgs{"KEEP_LAST", "KEEP_ALL"};
    TCLAP::ValuesConstraint<std::string> allowedHistoryArgsVals(allowedHistoryArgs);
    TCLAP::ValueArg<std::string> historyArg("", "history",
      "The QOS History type. Default is KEEP_ALL.", false, "KEEP_ALL",
      &allowedHistoryArgsVals, cmd);

    TCLAP::ValueArg<uint32_t> historyDepthArg("", "history-depth",
      "The history depth QOS. Default is 1000.", false, 1000, "N", cmd);

    TCLAP::ValueArg<uint64_t> maxRuntimeArg("", "max-runtime",
      "Run N seconds, then exit. 0 means run forever. Default is 0.", false, 0, "N", cmd);

    std::vector<uint32_t> allowedNumPubsArgs{0, 1};
    TCLAP::ValuesConstraint<uint32_t> allowedNumPubsArgsVals(allowedNumPubsArgs);
    TCLAP::ValueArg<uint32_t> numPubsArg("p", "num-pub-threads",
      "Number of publisher threads. Default is 1.", false, 1,
      &allowedNumPubsArgsVals, cmd);

    TCLAP::ValueArg<uint32_t> numSubsArg("s", "num-sub-threads",
      "Number of subscriber threads. Default is 1.", false, 1, "N", cmd);

    TCLAP::SwitchArg checkMemoryArg("", "check-memory",
      "Print backtrace of all memory operations performed by the middleware. "
      "This will slow down the application!", cmd, false);

    TCLAP::ValueArg<int32_t> useRtPrioArg("", "use-rt-prio",
      "Set RT priority using a SCHED_FIFO real-time policy. "
      "This option requires permissions to set a real-time priority. "
      "Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> useRtCpusArg("", "use-rt-cpus",
      "Set RT CPU affinity mask. "
      "The affinity mask has to be in decimal system. "
      "For example, 10 sets the affinity for processors 1 and 3. "
      "Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::SwitchArg withSecurityArg("", "with-security",
      "Make nodes with deterministic names for use with security.", cmd, false);

    std::vector<std::string> allowedRoundTripModes{{"None", "Main", "Relay"}};
    TCLAP::ValuesConstraint<std::string> allowedRoundTripModeVals(allowedRoundTripModes);
    TCLAP::ValueArg<std::string> roundTripModeArg("", "roundtrip-mode",
      "Select the round trip mode. Default is None.", false, "None",
      &allowedRoundTripModeVals, cmd);

    TCLAP::ValueArg<uint32_t> ignoreArg("", "ignore",
      "Ignore the first N seconds of the experiment. Default is 0.", false, 0, "N", cmd);

    std::vector<uint32_t> allowedExpectedNumPubsArgs{0, 1};
    TCLAP::ValuesConstraint<uint32_t> allowedExpectedNumPubsArgsVals(allowedExpectedNumPubsArgs);
    TCLAP::ValueArg<uint32_t> expectedNumPubsArg("", "expected-num-pubs",
      "Expected number of publishers for wait-for-matched. Default is 0.", false, 0,
      &allowedExpectedNumPubsArgsVals, cmd);

    TCLAP::ValueArg<uint32_t> expectedNumSubsArg("", "expected-num-subs",
      "Expected number of subscribers for wait-for-matched. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> waitForMatchedTimeoutArg("", "wait-for-matched-timeout",
      "Maximum time in seconds to wait for matched pubs/subs. Default is 30.", false, 30, "N", cmd);

    TCLAP::SwitchArg zeroCopyArg("", "zero-copy",
      "Use zero copy transfer.", cmd, false);

    TCLAP::ValueArg<uint32_t> unboundedMsgSizeArg("", "unbounded-msg-size",
      "The number of bytes to use for an unbounded message type. "
      "Ignored for other messages. Default is 0.",
      false, 0, "N", cmd);

    TCLAP::SwitchArg preventCpuIdleArg("", "prevent-cpu-idle",
      "Prevent CPU from entering idle states.", cmd, false);

    cmd.parse(argc, argv);

    QOSAbstraction qos;
    qos.reliability = qos_reliability_from_string(reliabilityArg.getValue());
    qos.durability = qos_durability_from_string(durabilityArg.getValue());
    qos.history_kind = qos_history_kind_from_string(historyArg.getValue());
    qos.history_depth = historyDepthArg.getValue();

    RealTimeConfiguration rt_config;
    rt_config.prio = useRtPrioArg.getValue();
    rt_config.cpus = useRtCpusArg.getValue();

    OutputConfiguration output_config;
    output_config.print_to_console = printToConsoleArg.getValue();
    output_config.logfile_path = LogfileArg.getValue();

    experiment_configuration.communicator = communicatorArg.getValue();
    experiment_configuration.execution_strategy = executionStrategyArg.getValue();
    experiment_configuration.dds_domain_id = ddsDomainIdArg.getValue();
    experiment_configuration.qos = qos;
    experiment_configuration.rate = rateArg.getValue();
    experiment_configuration.topic_name = topicArg.getValue();
    experiment_configuration.msg_name = msgArg.getValue();
    experiment_configuration.unbounded_msg_size = unboundedMsgSizeArg.getValue();
    experiment_configuration.max_runtime = maxRuntimeArg.getValue();
    experiment_configuration.rows_to_ignore = ignoreArg.getValue();
    experiment_configuration.number_of_publishers = numPubsArg.getValue();
    experiment_configuration.number_of_subscribers = numSubsArg.getValue();
    experiment_configuration.expected_num_pubs = expectedNumPubsArg.getValue();
    experiment_configuration.expected_num_subs = expectedNumSubsArg.getValue();
    experiment_configuration.wait_for_matched_timeout =
      std::chrono::seconds(waitForMatchedTimeoutArg.getValue());
    experiment_configuration.check_memory = checkMemoryArg.getValue();
    experiment_configuration.rt_config = rt_config;
    experiment_configuration.with_security = withSecurityArg.getValue();
    experiment_configuration.is_zero_copy_transfer = zeroCopyArg.getValue();
    experiment_configuration.prevent_cpu_idle = preventCpuIdleArg.getValue();
    experiment_configuration.roundtrip_mode =
      round_trip_mode_from_string(roundTripModeArg.getValue());
    experiment_configuration.output_configuration = output_config;
    experiment_configuration.argc = argc;
    experiment_configuration.argv = argv;
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }
}

}  // namespace performance_test
