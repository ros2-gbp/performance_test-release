# Changelog for package performance_test

## X.Y.Z (YYYY/MM/DD)

## 2.3.0 (2024/09/24)

### Removed
- Moved `apex_performance_plotter` to its own package [here](../plotter)

## 2.2.0 (2024/05/15)

### Added
- performance_test can be built with ROS 2 Iron and Jazzy
### Changed
- Renamed the `--dds-domain_id` CLI arg to `--dds-domain-id`
- When `--dds-domain-id` is unspecified, fall back to the `ROS_DOMAIN_ID` environment variable
- `--zero-copy` has been separated into two flags:
  - `--shared-memory`: Enable shared-memory transfer in the plugin. This is meant to replace the
    need to manually set runtime flags via `CYCLONEDDS_URI`, `APEX_MIDDLEWARE_SETTINGS`, etc.
  - `--loaned-samples`: When publishing messages in the plugin, borrow loaned samples instead
    of publishing by copy
  - `--zero-copy` is now an alias for `--shared-memory --loaned-samples`
  - Supported plugins include:
    - `-c CycloneDDS`
    - `-c CycloneDDS-CXX`
    - `-c ApexOSPollingSubscription`
    - `-c rclcpp-*` with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    - `-c rclcpp-*` with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

## 2.1.0 (2024/04/17)

### Added
- Add new function `prepare()` to the Publisher and Subscriber API, intended
  to allow participant discovery without blocking the main thread
### Changed
- Change the default `--history` arg from `KEEP_ALL` to `KEEP_LAST`
- Change the default `--history-depth` arg from `1000` to `16`
- If `--expected-num-pubs` is unspecified, set it to the same value as `-p`
- If `--expected-num-subs` is unspecified, set it to the same value as `-s`
### Fixed
- Removed an unused variable to fix a Clang build
- Remove unused variable names in the `Plugin` abstract class
- Fix a potential lockup in PublisherTask on QNX

## 2.0.0 (2024/03/19)

### Added
- Add experimental bazel support
   - `bazel build //performance_test --//:plugin_implementation=//path/to/a/plugin`
- Add a rudimentary socket-based plugin for testing the bazel support
   - `bazel run //performance_test --//:plugin_implementation=//performance_test/plugins/demo:demo_plugin -- --help`
### Changed
- Instead of enabling/disabling each plugin, you select exactly one
  with a CMake string option, for example:
   - `colcon build --cmake-args -DPERFORMANCE_TEST_PLUGIN=ROS2`
- Renamed the `--communication` CLI arg to `--communicator`. The short `-c` is unchanged.
### Removed
- Removed the deprecated CLI flags for QOS settings:
  - Instead of `--reliable`, use `--reliability RELIABLE`
  - Instead of `--transient`, use `--durability TRANSIENT_LOCAL`
  - Instead of `--keep-last`, use `--history KEEP_LAST`
- Removed the obsolete `BoundedSequenceFlat` messages
- Removed the superfluous `--msg-list` CLI flag. The `--help` message
  already lists the available messages.
### Fixed
- Update the Apex.OS Runner to use `executor_runner::deferred` instead of `executor_runner::deferred_tag()`
- Ensure that the first few published samples are sent at the expected rate

## 1.5.2 (YYYY/MM/DD)

### Added
- `--prevent-cpu-idle` is available on QNX
### Changed
- JSON log files will contain all values in the `APEX_PERFORMANCE_TEST` dictionary, instead of the
  five specific values used previously
- Switch to build as C++17 by default
### Fixed
- Zero copy transfer is again enabled for the rclcpp publisher

## 1.5.0 (2023/06/14)

### Added
- New CLI switch `--prevent-cpu-idle` (linux only). When specified,
  perf_test will use `/dev/cpu_dma_latency` to request that the CPU not enter
  any sleep states, to potentially give more consistent results
- Some smaller `Array` messages, down to 32 bits
- Added support to the FastDDS plugin for bounded and unbounded sequences
### Changed
- Update the README to better explain how to use this tool with Apex.OS
- In the `Runner`, allocate the `AnalysisResult`s on the stack instead of using `shared_ptr`
- `Subscriber` methods accept a callback parameter, instead of returning a
  `vector` of results, to reduce heap usage
- Refactored the interaction between `SubscriberStats` and `AnalysisResult` to
  remove the need for a `std::vector` of latency samples, to reduce heap usage
- Adjusted the `Array` message sizes to make the name match the contents
- Updated `apex_os_communicator` to use the new zero-copy API

## 1.4.2 (2023/03/15)

### Added
- Added `perfplot` support for JSON log files
### Changed
- Migrate the Apex.OS target to use `rosidl_get_typesupport_target`
- Preallocate the JSON logger's string buffer to prevent reallocations after the experiment begins

## 1.4.1 (2023/02/23)

### Changed
- Updated the iceoryx plugin to the [latest master as of Feb 13](https://github.com/eclipse-iceoryx/iceoryx/tree/5fe215eab267b0ad68bc5552aecd4b6e728e4c99)

## 1.4.0 (2023/02/20)

### Added
- New message type `BoundedSequenceFlat`
   - This is a `BoundedSequence` with the `@flat` annotation
   - Sizes range from 1kB to 8MB, like `Array` and `BoundedSequence`
### Changed
- Messages of different types can be optionally included via CMake args:
   - `-DENABLE_MSGS_ARRAY` (default ON)
   - `-DENABLE_MSGS_STRUCT` (default ON)
   - `-DENABLE_MSGS_POINT_CLOUD` (default ON)
   - `-DENABLE_MSGS_BOUNDED_SEQUENCE` (default OFF)
   - `-DENABLE_MSGS_BOUNDED_SEQUENCE_FLAT` (default OFF)
   - `-DENABLE_MSGS_UNBOUNDED_SEQUENCE` (default OFF)
   - `-DENABLE_MSGS_ALL` (default OFF)
      - when ON, overrides the other defaults to ON
      - you can still optionally exclude some messages by explicitly setting them to OFF
### Removed
- Removed a few messages:
   - Range
   - RadarTrack
   - RadarDetection
   - NavSatFix
### Fixed
- In all cases, including loaned messages, capture the timestamp as the last step of initializing the message

## 1.3.7 (2023/01/04)

## 1.3.6 (2023/01/03)

### Fixed
- Set the correct `IDL_GEN_ROOT` for rclcpp plugins

## 1.3.5 (2022/12/05)

### Fixed
- Exit cleanly when a publisher process terminates before a subscriber process

## 1.3.4 (2022/11/28)

### Changed
- Updated Apex.OS plugins to use the unified `LoanedSample::data()`

## 1.3.3 (2022/11/28)

### Fixed
- Implement the missing `take()` method in `ApexOSPollingSubscriptionSubscriber`

## 1.3.2 (2022/11/21)

### Fixed
- Capture the `this` pointer in the lambda in the iceoryx publisher

## 1.3.1 (2022/11/21)

### Added
- New Apex.OS plugin, compatible with the `ThreadedRunner`s
  - The `INTER_THREAD` and `INTRA_THREAD` execution strategies, combined with
    `-c ApexOSPollingSubscription`, will use the `ThreadedRunner` instances
  - The new `APEX_SINGLE_EXECUTOR` execution strategy will add all publishers
    and subscribers to a single Apex.OS Executor
  - The new `APEX_EXECUTOR_PER_COMMUNICATOR` execution strategy will add each
    publisher and each subscriber to its own Apex.OS Executor instance
  - The new `APEX_CHAIN` execution strategy will add a publisher and subscriber
    as a chain of nodes to an Apex.OS Executor
### Changed
- Refactored FastRTPS communicator plugin:
  - Uses DDS compliant API
  - Code generator updated
  - Implementation for `publish_loaned()`
  - Dockerfile improvements
### Removed
- CLI arg `--disable-async`. Synchronous / asynchronous publishing should be
  configured externally depending on the communication mean used.

## 1.3.0 (2022/08/25)

### Added
- New execution strategy option:
  - The default `-e INTER_THREAD` runs each publisher and subscriber in its
    own separate thread, which matches the previous behavior
  - A new `-e INTRA_THREAD`, which runs a single publisher and subscriber in the
    same thread. The publisher writes, and the subscriber immediately takes it
  - For Apex.OS specifically, some optimized execution strategies which use the
    proprietary Apex.OS executor
### Changed
- Significantly refactored the communicator plugins:
  - Each plugin is split into an implementation of a `Publisher` and a
    `Subscriber`, instead of a single `Communicator`
  - The plugin is no longer responsible for managing the metrics, such as
    sample count, lost samples, and latency
  - The plugin does not require any special logic to support roundtrip mode
  - It is safe for the plugins to initialize their data writers and readers
    at construction time, instead of delaying the initialization to the first
    call of `publish()` or `update_subscription()`
  - Split `publish()` into `publish_copy()` and `publish_loaned()`
- Significantly refactored the runner framework:
  - The runner framework is responsible for the experiment metrics
  - It manages the roundtrip mode logic
  - It is extensible for different execution strategies or thread configurations
- The iceoryx plugin now uses the untyped API, for improved performance

## 1.2.1 (2022/06/30)

### Fixed
- Capture the timestamp as soon as a message is received, instead of just before
  storing the metrics, to reduce the reported latency to a more correct value

## 1.2.0 (2022/06/28)

### Changed
- The CLI arguments for specifying the output type have changed:
   - For console output, updated every second, add `--print-to-console`
   - For file output, use `--logfile my_file.csv` or `--logfile my_file.json`
      - The type will be deduced from the file name
   - If neither of these options is specified, then a warning will print,
     and the experiment will still run
- The linter configurations are now configured locally. This means that the output
  of `colcon test` should be the same no matter the installed ROS distribution.
- The `--zero-copy` arg is now valid even if the publisher and subscriber(s)
  are in the same process
### Removed
- The publisher and subscriber loop reserve metrics are no longer recorded or reported
### Fixed
- CPU usage will no longer be stuck at `0`

### Removed
- The pub/sub loop reserve time metrics

## 1.1.2 (2022/06/08)

### Changed
- Use `steady_clock` for all platforms, including QNX QOS

## 1.1.1 (2022/06/07)

### Changed
- Significant refactor to simplify the analysis pipeline
### Fixed
- Add some missing definitions when Apex.OS is enabled, but the rclcpp plugins are disabled

## 1.1.0 (2022/06/02)

### Added
- New Apex.OS Polling Subscription plugin
- Compatibility with ROS2 Humble

## 1.0.0 (2022/05/12)

### Added
- More expressive perf_test CLI args for QOS settings
- A plugin for [Cyclone DDS with C++ bindings v0.9.0b1](https://github.com/eclipse-cyclonedds/cyclonedds-cxx/tree/0.9.0b1)
### Changed
- CLI args for QOS settings:
    - `--reliability <RELIABLE|BEST_EFFORT>`
    - `--durability <TRANSIENT_LOCAL|VOLATILE>`
    - `--history <KEEP_LAST|KEEP_ALL>`
- `master` branch is compatible with many ROS2 distributions:
    - dashing
    - eloquent
    - foxy
    - galactic
    - rolling
### Deprecated
- CLI flags for QOS settings:
    - `--reliable`
    - `--transient`
    - `--keep-last`
### Removed
- The branches for specific ROS2 distributions have been deleted
### Fixed
- CI jobs and Dockerfiles are decoupled from the middleware bundled with the ROS2 distribution
