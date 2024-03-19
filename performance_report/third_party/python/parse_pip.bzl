# Copyright 2024 Apex.AI, Inc.
# All rights reserved.

load("@rules_python//python:pip.bzl", "pip_parse")

def parse_pip():
    pip_parse(
        name = "pip_deps",
        requirements_lock = "@performance_test//performance_report/third_party/python:requirements.txt",
    )
