load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@simulation_pip//:requirements.bzl", "requirement")

py_library(
    name = "kinematics",
    srcs = [
        "kinematics.py",
    ],
    deps = [
        requirement("numpy"),
        requirement("matplotlib"),
    ],
)

py_binary(
    name = "main",
    srcs = ["main.py"],
    data = ["//simulation:model"],
    deps = [":kinematics"],
)
