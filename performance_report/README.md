# performance_report

[TOC]

This package serves two purposes:

1. Run multiple `performance_test` experiments
2. Visualize the combined results of those experiments

## Quick start

Install the required dependencies:

```shell
python3 -m pip install bokeh selenium pandas
sudo apt-get install firefox-geckodriver
```

Note: all the commands below are run from the `colcon` workspace where
`performance_test/performance_report` is installed:

```shell
# Build performance_test and performance_report
colcon build

# Set up the environment
source install/setup.bash

# Run perf_test for each experiment in the yaml file
ros2 run performance_report runner \
  --log-dir perf_logs \
  --test-name experiments \
  --configs src/performance_test/performance_report/cfg/runner/run_one_experiment.yaml

# The runner generates log files to the specified directory: `./perf_logs/experiements/`

# Generate the plots configured in the specified yaml file
ros2 run performance_report plotter \
  --log-dir perf_logs \
  --configs src/performance_test/performance_report/cfg/plotter/plot_one_experiment.yaml

# The generated plots will be saved in `./perf_logs`

# Generate the reports configured in the specified yaml file
ros2 run performance_report reporter \
  --log-dir perf_logs \
  --configs src/performance_test/performance_report/cfg/reporter/report_one_experiment.yaml
```

## `runner`

The `performance_report runner` tool is a wrapper around `performance_test perf_test`.
It executes one or more `perf_test` experiments defined in a yaml file:

```yaml
---
experiments:
  -
    com_mean: ApexOSPollingSubscription  # or rclcpp-single-threaded-executor for ROS 2
    msg: Array1k
    rate: 20
  -
    com_mean: ApexOSPollingSubscription
    msg: Array4k
    rate: 20
```

To run all experiments in the config file, only a single command is required:

```shell
ros2 run performance_report runner \
  --configs input/path/to/config.yaml \
  --log-dir output/path/to/log/files \
  --test-name custom_name_for_this_set_of_tests
```

`runner` will invoke `perf_test` for each experiment, in sequence. The results for each
experiment will be stored in a json log file in the directory
`output/path/to/log/files/custom_name_for_this_set_of_tests/`.

For a list of all experiment configuration options, and their default values,
see any of the example yaml configuration files in [cfg/runner](cfg/runner).

`runner` will by default skip any experiments that already have
log files generated in the output directory. This can be overridden
by adding `-f` or `--force` to the command.

### Reducing duplication in configuration files

All of the experiment values can be a single value or an array:

```yaml
---
experiments:
  -
    com_mean: ApexOSPollingSubscription
    msg:
      - Array1k
      - Array4k
      - Array16k
    pubs: 1
    subs: 1
    rate:
      - 20
      - 500
    reliability:
      - RELIABLE
      - BEST_EFFORT
    durability:
      - VOLATILE
      - TRANSIENT_LOCAL
    history: KEEP_LAST
    history_depth: 16
```

For this configuration file, `runner` would run all combinations, for a total of
24 experiments.

YAML aliases and anchors are also a great way to reduce duplication:

```yaml
---
comparison_experiments_common: &comparison_experiments_common
  com_mean: ApexOSPollingSubscription
  msg:
    - Array1k
    - Array4k
    - Array16k
    - Array64k
    - Array256k
    - Array1m
    - Array4m
  rate: 20

inter_thread_copy: &inter_thread_copy
  process_configuration: INTRA_PROCESS
  execution_strategy: INTER_THREAD
  sample_transport: BY_COPY

inter_process_copy: &inter_process_copy
  process_configuration: INTER_PROCESS
  execution_strategy: INTER_THREAD
  sample_transport: BY_COPY

inter_process_loaned: &inter_process_loaned
  process_configuration: INTER_PROCESS
  execution_strategy: INTER_THREAD
  sample_transport: LOANED_SAMPLES

experiments:
  -
    <<: *comparison_experiments_common
    <<: *inter_thread_copy
  -
    <<: *comparison_experiments_common
    <<: *inter_process_copy
  -
    <<: *comparison_experiments_common
    <<: *inter_process_loaned
```

## `commander`

`commander` generates the `perf_test` commands that would be invoked
by `runner`, but does not actually run them:

```shell
ros2 run performance_report commander \
  --configs input/path/to/config.yaml \
  --log-dir output/path/to/log/files \
  --test-name custom_name_for_this_set_of_tests
```

The result (written to `stdout`) is a set of commands for invoking `perf_test`
directly, for all of the experiments in the configuration file. The output can
be inspected manually, or invoked:

```shell
ros2 run performance_report commander ...args... > perf_test_commands.sh
chmod +x perf_test_commands.sh
./perf_test_commands.sh
```

After invoking the generated script, the result is the same as if `runner` were
used originally.

## `plotter`

After experiments are complete, `plotter` can generate static images of plots
from the resulting data:

```shell
ros2 run performance_report plotter \
  --configs input/path/to/config.yaml \
  --log-dir input/path/to/log/files
```

The `plotter` configuration files are easiest to explain through example.
Example yaml configuration files can be found in [cfg/plotter](cfg/plotter).
Each is intended to be used with one of the example runner configurations, as
shown in the [Quick start](#quick-start) instructions above.

## `reporter`

While `plotter` can generate static images, `reporter` uses
[Jinja](jinja.palletsprojects.com) templates to create a markdown or html report
containing interactible [bokeh](bokeh.org) plots:

```shell
ros2 run performance_report reporter \
  --configs input/path/to/config.yaml \
  --log-dir input/path/to/log/files
```

The `reporter` configuration files are very similar to those for `plotter`,
and also are easiest to explain through example.
Example yaml configuration files can be found in [cfg/reporter](cfg/reporter).
Each is intended to be used with one of the example runner configurations, as
shown in the [Quick start](#quick-start) instructions above. Also see the
example `.md` and `.html` template files, from which the output reports
are generated.

## Running the same experiments on multiple platforms

Suppose you want to run an experiment on multiple platforms, then combine the results into a single
report. First, pass the `--test-name` arg to `runner`, to differentiate the result sets:

```shell
# on platform 1:
ros2 run performance_report runner --test_name platform1 -l log_dir -c run.yaml
# results will be stored in ./log_dir/platform1/

# on platform 2:
ros2 run performance_report runner --test_name platform2 -l log_dir -c run.yaml
# results will be stored in ./log_dir/platform2/
```

You can then combine these results into a single `log_dir`, on the platform where you will run
`plotter` or `reporter`. Then, in your `plotter` or `reporter` configuration file, set `test_name`
in each dataset, to select results from that platform's result set:

```yaml
# report.yaml
datasets:
  dataset_p1:
    test_name: platform1  # this matches the --test-name passed to runner
    # other fields...
  dataset_p2:
    test_name: platform2  # this matches the --test-name passed to runner
    # other fields...
reports:
  # ...
```

```shell
ros2 run performance_report reporter -l log_dir -c report.yaml
```

## Notes

- Currently, this tool is intended for ROS 2 with rmw_cyclone_dds, or Apex.OS with
  Apex.Middleware. It has not been tested with any other transport.
- If the run configuration includes `SHMEM` or `ZERO_COPY` transport, then a file for
  configuring the middleware will be created to enable the shared memory transfer.
  - You must start RouDi before running the experiments. This tool will not automatically
    start it for you.
