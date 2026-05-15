# Copyright 2024 Apex.AI, Inc.
# All rights reserved.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def load_repositories():
    http_archive(
        name = "rapidjson",
        urls = ["https://github.com/Tencent/rapidjson/archive/v1.1.0.tar.gz"],
        sha256 = "bf7ced29704a1e696fbccf2a2b4ea068e7774fa37f6d7dd4039d0787f8bed98e",
        strip_prefix = "rapidjson-1.1.0",
        build_file = "@performance_test//performance_test/third_party/rapidjson:BUILD.bazel",
    )

    http_archive(
        name = "tclap",
        urls = ["https://github.com/mirror/tclap/archive/refs/tags/1.4.0-rc1.tar.gz"],
        sha256 = "54d9afd826edf05accd24b56b8a50c7da10e3eaeae0cff698599cd595f50dd6d",
        strip_prefix = "tclap-1.4.0-rc1",
        build_file = "@performance_test//performance_test/third_party/tclap:BUILD.bazel",
    )

    http_archive(
        name = "sole",
        urls = ["https://github.com/r-lyeh-archived/sole/archive/refs/tags/1.0.2.tar.gz"],
        sha256 = "ff82a1d6071cbc9c709864266210ddedecdb2b1e507ac5e7c4290ca6453e89b3",
        strip_prefix = "sole-1.0.2",
        build_file = "@performance_test//performance_test/third_party/sole:BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@performance_test//performance_test/third_party/sole:diff.patch"],
        patch_cmds = [
            "mkdir -p include/sole",
            "mv sole.hpp include/sole",
        ],
    )

    http_archive(
        name = "tabulate",
        urls = ["https://github.com/p-ranav/tabulate/archive/refs/tags/v1.4.tar.gz"],
        sha256 = "c20cdc3175526a069e932136a7cbdf6f27b137bdb4fc5f574eb5a497228c8e11",
        strip_prefix = "tabulate-1.4",
        build_file = "@performance_test//performance_test/third_party/tabulate:BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@performance_test//performance_test/third_party/tabulate:diff.patch"],
    )

    http_archive(
        name = "rules_python",
        sha256 = "e85ae30de33625a63eca7fc40a94fea845e641888e52f32b6beea91e8b1b2793",
        strip_prefix = "rules_python-0.27.1",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.27.1.tar.gz",
    )
