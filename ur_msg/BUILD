load("@com_nvidia_isaac_engine//engine/build:cc_capnp_library.bzl", "cc_capnp_library")
load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")
load("//bzl:pybind.bzl", "pybind_library")

cc_capnp_library(
    name = "speed_slider_proto",
    protos = [
        "speed_slider.capnp"
    ],
)

cc_capnp_library(
    name = "dashboard_command_proto",
    protos = [
        "dashboard_command.capnp"
    ],
)

isaac_cc_library(
    name = "dashboard_command",
    srcs = [
        "dashboardCommand.cpp"
    ],
    hdrs = [
        "dashboardCommand.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dashboard_command_proto",
        "@com_nvidia_isaac_engine//engine/core",
    ],
)

py_library(
	name = "ur_msg_python",
	srcs = [
		"ur_msg_python.py"
	],
	visibility = ["//visibility:public"]
)