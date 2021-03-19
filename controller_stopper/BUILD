load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "controller_stopper",
    srcs = [
        "ControllerStopper.cpp"
    ],
    hdrs = [
        "ControllerStopper.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/alice",
        "//messages:basic_proto"
    ],
)