load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@teleop_pip//:requirements.bzl", "requirement")

py_binary(
    name = "test_example",
    srcs = [
        "test_example.py",
    ],
    deps = [":input_controller"],
)

py_library(
    name = "input_controller",
    srcs = [
        "input_controller.py",
        "event_handler.py",
        "device_manager.py",
    ],
    deps = [
        requirement("evdev"),
    ],
)
