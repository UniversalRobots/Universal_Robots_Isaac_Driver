load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_test_group")

isaac_cc_test_group(
	srcs = glob(["*.cpp"]),
	deps = [
       	"//packages/universal_robots/ur_controller:ur_controller_test",
    	"//packages/composite/gems:serializer",
  	],
  	data = [
    	"//apps/assets/kinematic_trees"
 	]
)