# Changelog for package performance_report

## X.Y.Z (YYYY/MM/DD)

## 2.0.0 (2024/03/19)

### Removed
- Removed the special handling for the `BoundedSequenceFlat` messages, because
  the messages are removed in performance_test

## 1.5.2 (2024/01/24)

### Fixed
- Elegantly handle a failure to parse JSON log files

## 1.5.0 (2023/06/14)

### Added
- The `reporter` box-and-whisker latency plots now support `latency_mean_ms`
  for the y-axis, in addition to the previously-supported `latency_mean`
- Added a new option `prevent_cpu_idle` (bool) for experiment configurations,
  which corresponds to the `--prevent-cpu-idle` switch in `perf_test`
### Changed
- Update the README to better explain the purpose and usage of
  `runner`, `commander`, `plotter`, and `reporter`

## 1.4.2 (2023/03/15)

## 1.4.1 (2023/02/23)

## 1.4.0 (2023/02/20)

### Added
- Figures have a new `x_range` option: `ru_maxrss_mb`
### Changed
- `BoundedSequenceFlatXYZ` will be mapped to `BoundedSequenceXYZ` for categorical plots,
  so that both message types can be compared directly on a single plot

## 1.3.7 (2023/01/04)

### Added
- The `reporter` templates can access os environment variables:
   - `{{ env['SOME_ENVIRONMENT_VARIABLE'] }}`
-  For error detection, the exit code for performance_report `reporter` is the number of missing datasets

### Fixed
- Add a workaround for [bug in bokeh](https://github.com/bokeh/bokeh/issues/12414)

## 1.3.6 (2023/01/03)

## 1.3.5 (2022/12/05)

## 1.3.4 (2022/11/28)

## 1.3.3 (2022/11/28)

### Fixed
- Do not try to create a box-and-whisker for a file that contains no measurements

## 1.3.2 (2022/11/21)

## 1.3.1 (2022/11/21)

## 1.3.0 (2022/08/25)

### Added
- The `reporter` configuration supports box-and-whisker latency plots:
   - set the `x_range` to `Experiment`
   - set the `y_range` to `latency_mean`
   - set `datasets` to one or more datasets, each containing a single experiment
   - an example can be found in `cfg/reporter/report_many_experiments.yaml`
### Changed
- Expanded the `transport` setting into the following two settings:
   - `process_configuration`:
      - `INTRA_PROCESS`
      - `INTER_PROCESS`
   - `sample_transport`:
      - `BY_COPY`
      - `SHARED_MEMORY`
      - `LOANED_SAMPLES`

## 1.2.1 (2022/06/30)

## 1.2.0 (2022/06/28)

### Changed
- In the `reporter` configuration, the `template_name` value may be an array

## 1.1.2 (2022/06/08)

## 1.1.1 (2022/06/07)

### Fixed
- Bokeh line style can be specified in the plotter and reporter .yaml files

## 1.1.0 (2022/06/02)

### Fixed
- Fix the GBP builds by removing `python3-bokeh-pip` from package.xml

## 1.0.0 (2022/05/12)

### Added
- Shared memory experiments are now compatible with both Apex.Middleware and rmw_cyclonedds_cpp
- `commander` tool to emit the commands for running the experiments, instead of running them directly
### Changed
- Use the new perf_test CLI args for QOS settings instead of old flags
### Deprecated
### Removed
### Fixed
