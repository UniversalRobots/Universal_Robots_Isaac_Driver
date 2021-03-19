load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "ur_eseries_robot_subgraph",
    modules = [
        "//packages/universal_robots:universal_robots",
        "map",
        "planner",
        "lqr",
		"//packages/universal_robots/ur_controller:ur_controller", 
    ],
	data = [
		"//apps/assets/kinematic_trees",
	],
    subgraph = "ur_eseries_robot.subgraph.json",
    visibility = ["//visibility:public"],    
)

isaac_subgraph(
    name = "ur_cb3_robot_subgraph",
    modules = [
        "//packages/universal_robots:universal_robots",
        "map",
        "planner",
        "lqr",
		"//packages/universal_robots/ur_controller:ur_controller", 
    ],
	data = [
		"//apps/assets/kinematic_trees",
	],
    subgraph = "ur_cb3_robot.subgraph.json",
    visibility = ["//visibility:public"],    
)