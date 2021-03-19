load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")
load("//bzl:module.bzl", "isaac_cc_module",)

isaac_cc_module(
    name = "ur_controller",
    srcs = [
        "ScaledMultiJointController.cpp",
        "JointSegment.cpp",
        "JointTrajectory.cpp"
    ],
    hdrs = [
        "ScaledMultiJointController.hpp",
        "JointSegment.hpp",
        "Pid.hpp",
        "JointTrajectory.hpp"
    ],
    deps=[
        "//packages/map:kinematic_tree",
        "//packages/composite/gems:serializer",
        "//packages/composite/gems:parser"
    ]
)

isaac_cc_library(
    name = "ur_controller_test",
    srcs = [
        "ScaledMultiJointController.cpp",
        "JointSegment.cpp",
        "JointTrajectory.cpp"
    ],
    hdrs = [
        "ScaledMultiJointController.hpp",
        "JointSegment.hpp",
        "Pid.hpp",
        "JointTrajectory.hpp"
    ],
    deps=[
        "//packages/map:kinematic_tree",
        "//packages/composite/gems:parser"
    ],
    visibility = ["//visibility:public"],
)