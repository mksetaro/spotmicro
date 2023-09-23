load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@teleop_pip//:requirements.bzl", "requirement")

py_binary(
    name = "teleop",
    srcs = [
        "teleop.py",
    ],
    deps = [
        requirement("evdev")
    ],
)
