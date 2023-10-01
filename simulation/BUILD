load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@simulation_pip//:requirements.bzl", "requirement")

py_binary(
    name = "example",
    srcs = ["example.py"],
    data = [":model"],
    deps = [":spotmicro"],
)

filegroup(
    name = "model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
)

py_library(
    name = "spotmicro",
    srcs = [
        "kinematics.py",
        "spotmicroai.py",
    ],
    deps = [
        requirement("pybullet"),
        requirement("numpy"),
        requirement("matplotlib"),
    ],
)

py_library(
    name = "simulation",
    srcs = [
        "simulation.py",
    ],
    deps = [
        requirement("pybullet"),
        requirement("numpy"),
    ],
)

py_binary(
    name = "simulation_main",
    srcs = ["simulation_main.py"],
    data = [":model"],
    deps = [":simulation"],
)