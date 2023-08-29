load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@simulation_pip//:requirements.bzl", "requirement")
py_binary(
    name = "example",
    deps = [":spotmicro"],
    srcs = ["example.py",],
    data = [":model"],
)

filegroup(
    name = "model",
    srcs = glob(["robot_model/*"]),
)
py_library(
    name = "spotmicro",
    deps = [
            requirement("pybullet"),
            requirement("numpy"), 
            requirement("matplotlib")],
    srcs = [
            "spotmicroai.py",
            "kinematics.py"],
)