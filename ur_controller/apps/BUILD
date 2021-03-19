load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "scaled_multi_joint_ur_control_subgraph",
    modules = [
        "map",
        "planner",
        "lqr",
        "//packages/universal_robots/ur_controller:ur_controller",  
    ],
    subgraph = "scaled_multi_joint_ur_control.subgraph.json",
    visibility = ["//visibility:public"],    
)