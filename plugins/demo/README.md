# Demo Plugin

This directory contains a thin plugin implementation. The code contains comments
to explain the API and the purpose of each public function.

You can explore the code to better understand the Plugin API, or even copy/paste
the entire directory as a starting point for your own Plugin. This demo plugin
uses a simple Unix socket, which can be entirely replaced by your middleware.

This plugin contains a bazel BUILD file, which demonstrates how to bundle the plugin
into a library target. To build and run:

```
cd perf_test_ws/src/performance_test
bazel run //performance_test --//:plugin_implementation=//performance_test/plugins/demo:demo_plugin -- --help
```
