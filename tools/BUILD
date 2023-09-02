load("@buildifier_prebuilt//:rules.bzl", "buildifier")
load("@rules_python//python:defs.bzl", "py_binary")
load("@pip_lint//:requirements.bzl", "requirement")

buildifier(
    name = "buildifier",
)

py_binary(
    name = "py_lint",
    srcs = ["py_lint.py"],
    deps = [requirement("black")],
)
