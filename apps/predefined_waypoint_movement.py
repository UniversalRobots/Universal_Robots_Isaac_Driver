import numpy as np
import argparse
import json
import time

from numpy.lib.twodim_base import tri

from isaac import Application, Cask
from packages.pyalice.Composite import create_composite_message

'''
An app that moves a UR robot, between 6 predefined waypoints, this app is based on the shuffle_box_hardware.
However, this app can be used for all robot models, with the correct configuration of the kinematic model.
This app doesn't assume any gripper is attached to the robot.
'''


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message with name as uuid'''
    msg = create_composite_message(quantities, values)
    msg.uuid = name
    return msg


def create_composite_atlas(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10.'''
    if len(joints) != 6:
        raise ValueError("UR robot should have 6 joints, got {}".format(len(joints)))

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

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description="Sortbot Demo")
    parser.add_argument("--cask", help="Path to output atlas", default="/tmp/shuffle_box_waypoints")
    parser.add_argument("--robot_model", help="Robot model.", choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur10e", "ur16e"], default="ur5e")
    parser.add_argument("--robot_ip", help="robot ip", default="192.168.56.101")
    parser.add_argument("--headless_mode", help="start driver with headless mode enabled or not",
                        default=False, type=lambda x: (str(x).lower() == 'true'))
    args = parser.parse_args()

    # Load driver subgraph and kinematic model
    robot_model = args.robot_model
    if robot_model == "ur3":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur3.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur3.kinematic.json"
    elif robot_model == "ur5":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur5.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur5.kinematic.json"
    elif robot_model == "ur10":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur10.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur10.kinematic.json"
    elif robot_model == "ur3e":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur3e.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur3e.kinematic.json"
    elif robot_model == "ur5e":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur5e.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur5e.kinematic.json"
    elif robot_model == "ur10e":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur10e.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur10e.kinematic.json"
    elif robot_model == "ur16e":
        ur_subgraph = "packages/universal_robots/ur_robot_driver/apps/ur16e.subgraph.json"
        kinematic_file = "packages/universal_robots/ur_robot_driver/config/ur16e.kinematic.json"
    else:
        raise Exception("Unknown robot model")

    # get joints
    joints = []
    with open(kinematic_file, 'r') as fd:
        kt = json.load(fd)
        for link in kt['links']:
            if 'motor' in link and link['motor']['type'] != 'constant':
                joints.append(link['name'])

    # create composite atlas
    create_composite_atlas(args.cask, joints)

    # Create and start the app
    app = Application(name="Predefined waypoint movement", modules=["sight"])
    # load bebavior subgraph. this contains the sequency behavior to move the arm between the waypoints
    app.load("packages/universal_robots/apps/predefined_waypoint_movement.subgraph.json", prefix="behavior")
    behavior_interface = app.nodes["behavior.interface"]["subgraph"]
    app.nodes["behavior.atlas"]["CompositeAtlas"].config.cask = args.cask

    app.load(filename=ur_subgraph, prefix="ur")

    # Load components
    ur_interface = app.nodes["ur.subgraph"]["interface"]
    ur_controller = app.nodes["ur.controller"]["ScaledMultiJointController"]
    ur_driver = app.nodes["ur.universal_robots"]["UniversalRobots"]

    # Configs
    ur_controller.config.control_mode = "joint position"
    ur_driver.config.control_mode = "joint position"
    ur_driver.config.robot_ip = args.robot_ip
    ur_driver.config.headless_mode = args.headless_mode


    # Edges
    app.connect(ur_interface, "arm_state", behavior_interface, "joint_state")
    app.connect(behavior_interface, "joint_target", ur_interface, "joint_target")

    # Sequence nodes
    sequence_behavior = app.nodes["behavior.sequence_behavior"]
    repeat_behavior = app.nodes["behavior.repeat_behavior"]
    nodes_stopped = True

    # Enable sight
    widget = app.add("sight").add(app.registry.isaac.sight.SightWidget, "predefined_waypoint_movement")
    widget.config.type = "plot"
    widget.config.channels = [
        {
            "name": "ur.universal_robots/UniversalRobots/shoulder_pan_joint"
        },
        {
            "name": "ur.universal_robots/UniversalRobots/shoulder_lift_joint"
        },
        {
            "name": "ur.universal_robots/UniversalRobots/elbow_joint"
        },
        {
            "name": "ur.universal_robots/UniversalRobots/wrist_1_joint"
        },
        {
            "name": "ur.universal_robots/UniversalRobots/wrist_2_joint"
        },
        {
            "name": "ur.universal_robots/UniversalRobots/wrist_4_joint"
        }
    ]

    # run app
    app.start()

    try:
        while True:
            # Make sure urcap is running when starting the nodes
            program_running = app.receive("ur.subgraph", "interface", "robot_program_running")
            if program_running is None:
                time.sleep(1)
            else:
                if program_running.proto.flag == True and nodes_stopped:
                    sequence_behavior.start()
                    repeat_behavior.start()
                    nodes_stopped = False
                elif program_running.proto.flag == False and nodes_stopped == False:
                    sequence_behavior.stop()
                    repeat_behavior.stop()
                    nodes_stopped = True

    except KeyboardInterrupt:
        if nodes_stopped == False:
            sequence_behavior.stop()
            repeat_behavior.stop()
        app.stop()