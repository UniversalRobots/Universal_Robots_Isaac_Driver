load("//bzl:py.bzl", "isaac_jupyter_app", "isaac_py_app")
load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "shuffle_box_behavior_hardware_subgraph",
    modules = [
        "behavior_tree",
        "composite",
    ],
    subgraph = "shuffle_box_behavior_hardware.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "predefined_waypoint_movement_subgraph",
    modules = [
        "behavior_tree",
        "composite",
    ],
    subgraph = "predefined_waypoint_movement.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_jupyter_app(
    name = "simple_joint_control",
    data = [
        "//packages/universal_robots/ur_robot_driver/apps:ur3_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur5_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur10_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur3e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur5e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur10e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur16e_subgraph",
    ],
    modules = [
        "sight",
        "universal_robots",
    ],
    notebook = "simple_joint_control.ipynb",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "shuffle_box_hardware",
    srcs = ["shuffle_box_hardware.py"],
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/universal_robots/apps:shuffle_box_behavior_hardware_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur10_subgraph"
    ],
    modules = [
        "behavior_tree",
        "planner",
        "sight",
        "universal_robots",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "predefined_waypoint_movement",
    srcs = ["predefined_waypoint_movement.py"],
    data = [
        "//packages/universal_robots/apps:predefined_waypoint_movement_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur3_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur5_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur10_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur3e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur5e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur10e_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur16e_subgraph",
    ],
    modules = [
        "behavior_tree",
        "planner",
        "sight",
        "universal_robots",
    ],
    deps = [
        "//packages/pyalice",
    ],
)