import numpy as np
import argparse
import json

from engine.pyalice import Application, Cask
from engine.pyalice.Composite import create_composite_message
'''
Moves a robot arm based on joint waypoints to pickup and dropoff a box between two pre-defined
locations repeatedly. In the UR10 use case it also visualizes 3d pose estimation of KLTSmall
boxes in Sight, though the perception result is not used in motion control. This is tested with
omniverse kit isaac sim.
'''


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message with name as uuid'''
    msg = create_composite_message(quantities, values)
    msg.uuid = name
    return msg


def create_composite_atlas(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10. Tested with ovkit sim'''
    if len(joints) != 6:
        raise ValueError("UR10 should have 6 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # at joint waypoints
    quantities = [[x, "position", 1] for x in joints]

    CART_OBSERVE_WAYPOINT = np.array(
        [3.2334, -1.4856, 1.7979, 4.8503, -1.5219, 0.5585], dtype=np.dtype("float64"))
    CART_ALIGN_WAYPOINT = np.array(
        [3.3922, -0.8047, 1.7275, 3.7742, -1.5163, 1.8773], dtype=np.dtype("float64"))
    CART_DROPOFF_WAYPOINT = np.array(
        [3.3942, -0.6317, 1.6605, 3.6562, -1.5213, 1.8773], dtype=np.dtype("float64"))

    DOLLY_OBSERVE_WAYPOINT = np.array(
        [6.0535, -1.0559, 1.2437, 5.2903, -1.7218, 0.3887], dtype=np.dtype("float64"))
    DOLLY_ALIGN_WAYPOINT = np.array(
        [5.8864, -0.5826, 1.5502, 3.8685, -1.5238, 1.1526], dtype=np.dtype("float64"))
    DOLLY_DROPOFF_WAYPOINT = np.array(
        [5.8894, -0.3856, 1.4512, 3.7055, -1.5238, 1.1526], dtype=np.dtype("float64"))

    cask.write_message(create_composite_waypoint("cart_observe", quantities, CART_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_align", quantities, CART_ALIGN_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_dropoff", quantities, CART_DROPOFF_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_observe", quantities, DOLLY_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("dolly_align", quantities, DOLLY_ALIGN_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_dropoff", quantities, DOLLY_DROPOFF_WAYPOINT))

    quantities = [[x, "none", 1] for x in ["pump", "valve", "gripper"]]
    SUCTION_ON_WAYPOINT = np.array([1.0, 0.0, 1.0], dtype=np.dtype("float64"))
    SUCTION_OFF_WAYPOINT = np.array([0.0, 1.0, 0.0], dtype=np.dtype("float64"))
    VALVE_OFF_WAYPOINT = np.array([0.0, 0.0, 0.0], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("suction_on", quantities, SUCTION_ON_WAYPOINT))
    cask.write_message(create_composite_waypoint("suction_off", quantities, SUCTION_OFF_WAYPOINT))
    cask.write_message(create_composite_waypoint("valve_off", quantities, VALVE_OFF_WAYPOINT))


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description="Sortbot Demo")
    parser.add_argument("--cask", help="Path to output atlas", default="/tmp/shuffle_box_waypoints")
    args = parser.parse_args()

    # get kinematic file and joints
    kinematic_file = "apps/assets/kinematic_trees/ur10.kinematic.json"
    joints = []
    with open(kinematic_file, 'r') as fd:
        kt = json.load(fd)
        for link in kt['links']:
            if 'motor' in link and link['motor']['type'] != 'constant':
                joints.append(link['name'])

    # create composite atlas
    create_composite_atlas(args.cask, joints)

    # Create and start the app
    app = Application(name="Shuffle Box Hardware")
    # load bebavior subgraph. this contains the sequency behavior to move the arm between
    # waypoints and control suction on/off
    app.load("apps/samples/manipulation/shuffle_box_behavior.subgraph.json", prefix="behavior")
    behavior_interface = app.nodes["behavior.interface"]["subgraph"]
    app.nodes["behavior.atlas"]["CompositeAtlas"].config.cask = args.cask
    app.load("packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr")

    # load multi joint lqr control subgraph
    lqr_interface = app.nodes["lqr.subgraph"]["interface"]
    kinematic_tree = app.nodes["lqr.kinematic_tree"]["KinematicTree"]
    lqr_planner = app.nodes["lqr.local_plan"]["MultiJointLqrPlanner"]
    app.connect(behavior_interface, "joint_target", lqr_interface, "joint_target")
    kinematic_tree.config.kinematic_file = kinematic_file
    lqr_planner.config.speed_min = [-1.0] * len(joints)
    lqr_planner.config.speed_max = [1.0] * len(joints)
    lqr_planner.config.acceleration_min = [-1.0] * len(joints)
    lqr_planner.config.acceleration_max = [1.0] * len(joints)

    # load hardware subgraph
    app.load_module("universal_robots")
    arm = app.add("hardware").add(app.registry.isaac.universal_robots.UniversalRobots)
    arm.config.robot_ip = "10.32.221.190"
    arm.config.control_mode = "joint position"
    arm.config.tick_period = '125Hz'
    arm.config.kinematic_tree = "lqr.kinematic_tree"
    arm.config.tool_digital_out_names = ["valve", "pump"]
    arm.config.tool_digital_in_names = ["unknown", "gripper"]

    app.connect(arm, "arm_state", lqr_interface, "joint_state")
    app.connect(arm, "arm_state", behavior_interface, "joint_state")
    app.connect(arm, "io_state", behavior_interface, "io_state")
    app.connect(lqr_interface, "joint_command", arm, "arm_command")
    app.connect(behavior_interface, "io_command", arm, "io_command")

    # visualize IO in sight
    widget_io = app.add("sight_widgets").add(app.registry.isaac.sight.SightWidget, "IO")
    widget_io.config.type = "plot"
    widget_io.config.channels = [
      {
        "name": "hardware/UniversalRobots/gripper"
      },
      {
        "name": "hardware/UniversalRobots/pump"
      },
      {
        "name": "hardware/UniversalRobots/valve"
      }
    ]

    # run app
    app.run()
