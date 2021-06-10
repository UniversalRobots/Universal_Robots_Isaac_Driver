load("//bzl:module.bzl", "isaac_cc_module")

filegroup(
    name = "universal_robots_resources",
    data = [
        "ur_robot_driver/resources/ros_control.urscript",
        "ur_robot_driver/resources/rtde_input_recipe.txt",
        "ur_robot_driver/resources/rtde_output_recipe.txt",
    ],
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "universal_robots",
    srcs = [
        "ur_robot_driver/UniversalRobots.cpp",
        "ur_robot_driver/DashboardClientIsaac.cpp",
        "ur_robot_driver/UrclLogHandler.cpp",
    ] + glob([
        "ur_client_library/src/**/*.cpp",
    ]),
    hdrs = [
        "ur_robot_driver/UniversalRobots.hpp",
        "ur_robot_driver/DashboardClientIsaac.hpp",
        "ur_robot_driver/UrclLogHandler.hpp",
    ] + glob([
        "ur_client_library/include/ur_client_library/**/*.h",
    ]),
    copts = ["-Ipackages/universal_robots/ur_client_library/include"],
    data = ["//packages/universal_robots:universal_robots_resources"],
    defines = ["ISAAC_BUILD"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/math/gems/kinematic_tree",
        "//packages/composite/gems:parser",
        "//packages/map:kinematic_tree",
        "@boost//:variant",
        "//packages/universal_robots/ur_msg:speed_slider_proto",
        "//packages/universal_robots/ur_msg:dashboard_command_proto",
        "//packages/universal_robots/ur_msg:dashboard_command",
        "//packages/universal_robots/controller_stopper:controller_stopper",
        "//packages/composite/gems:serializer",
    ],
)