load("//engine/build:isaac.bzl", "isaac_jupyter_app", "isaac_py_app")

isaac_jupyter_app(
    name = "simple_joint_control",
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
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
        "//apps/samples/manipulation:shuffle_box_behavior_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "behavior_tree",
        "planner",
        "sight",
        "universal_robots",
    ],
    deps = [
        "//engine/pyalice",
    ],
)
