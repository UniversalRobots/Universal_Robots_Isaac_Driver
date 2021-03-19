import unittest
import numpy as np
import time
import argparse
from utils.test_utils import do_dashboard_command, wait_for_new_message, wait_for_dc_mode

from packages.pyalice import Application, Message, Composite

"""
This test requires the external_control program from the utils folder.
"""

class ExternalControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ip = args.robotip
        robot = args.robot

        cls.app = Application(name="external_control_test")
        if robot == "e-series":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")
        elif robot == "cb3":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_cb3_robot.subgraph.json", prefix="ur")
        else: # default to eseries
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

        ur_controller = cls.app.nodes["ur.controller"]["ScaledMultiJointController"]
        ur_controller.config.control_mode = "joint position"

        ur_driver = cls.app.nodes["ur.universal_robots"]["UniversalRobots"]
        ur_driver.config.control_mode = "joint position"
        ur_driver.config.robot_ip = ip

        cls.app.start()

        cls.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        cls.position_parser = [[x, "position", 1] for x in cls.joint_names]

        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        do_dashboard_command(cls.app, "stop")
        cls.app.stop()

    def init_robot(self):
        """Make sure the robot is booted and ready to receive commands."""
        self.wait_for_driver(self)

        # Make sure ursim is brake released and powered on
        time.sleep(2) # DashboardClient needs to be up and running
        do_dashboard_command(self.app, "stop")
        do_dashboard_command(self.app, "load", arguments="external_control.urp", timeout=15)
        do_dashboard_command(self.app, "brakeRelease")

        wait_for_dc_mode(self.app, "robotmode", "RUNNING")

        time.sleep(5)

    def wait_for_driver(self, timeout=30):
        """Make sure the driver is started and publishing robot states."""
        state_msg = wait_for_new_message(self.app, "ur.subgraph", "interface", "arm_state", timeout=timeout)
        if state_msg is None:
            self.fail("Could not receive message from driver. Make sure the driver is actually running")

    def wait_for_trajectory_result(self, timeout=20):
        """Wait for result of executed trajectory."""
        msg = wait_for_new_message(self.app, "ur.controller", "ScaledMultiJointController",
                                    "trajectory_executed_succesfully", timeout=timeout)
        if msg is None:
            self.fail("Could not read trajectory result within timeout {}".format(timeout))

        return msg.proto

    def wait_for_robot_program_running(self, state, timeout=20):
        """Wait for external program node to represent state."""
        wait_time = 0.0
        robot_program_running = None
        while wait_time < timeout:
            robot_program_running = self.app.receive("ur.subgraph", "interface", "robot_program_running")
            if robot_program_running is not None:
                if robot_program_running.proto.flag == state:
                    break

            time.sleep(0.1)
            wait_time += 0.1

        if wait_time >= timeout or robot_program_running is None:
            self.fail("Could not read desired program running state within timeout {}".format(timeout))

        self.assertEqual(robot_program_running.proto.flag, state)

    def test_external_control(self):
        """Testing that it is possible to switch between robot control and driver control."""
        do_dashboard_command(self.app, "play")

        self.wait_for_robot_program_running(True)

        joint_values = [-1.0 for i in range(6)]
        target_pos = np.array(joint_values, dtype=np.float64)
        target_pos_msg = Composite.create_composite_message(self.position_parser, target_pos)
        self.app.publish("ur.subgraph", "interface", "joint_target", target_pos_msg)

        trajectory_executed_succesfully = self.wait_for_trajectory_result()
        self.assertTrue(trajectory_executed_succesfully.flag == True, "failed to execute trajectory succesfully")

        stop_control_msg = Message.create_message_builder("BooleanProto")
        stop_control_msg.proto.flag = True
        self.app.publish("ur.subgraph", "interface", "stop_control", stop_control_msg)

        self.wait_for_robot_program_running(False, timeout=1)

        self.wait_for_robot_program_running(True)
        joint_states_msg = wait_for_new_message(self.app, "ur.subgraph", "interface", "arm_state")
        joint_states = Composite.parse_composite_message(joint_states_msg, self.position_parser)
        time.sleep(2)

        joint_command_msg = wait_for_new_message(self.app, "ur.controller", "ScaledMultiJointController", "joint_command")
        if joint_command_msg is None:
            self.fail("Could not read joint command within timeout {}".format(2))
        joint_command = Composite.parse_composite_message(joint_command_msg, self.position_parser)

        self.assertTrue(np.allclose(joint_command, joint_states, rtol=0.001, atol=0.001),
                                    "current joint command {} is not equal to expected joint states {}".format(joint_command, joint_states))

        new_joint_command_msg = wait_for_new_message(self.app, "ur.controller", "ScaledMultiJointController", "joint_command")
        if joint_command_msg is None:
            self.fail("Could not read joint command within timeout {}".format(2))
        new_joint_command = Composite.parse_composite_message(new_joint_command_msg, self.position_parser)

        self.assertTrue(np.allclose(joint_command, new_joint_command, rtol=0.001, atol=0.001),
                                    "current joint command {} and last joint command is not equal {}".format(new_joint_command, joint_command))

if __name__ == "__main__":
    # parse the arguments with --test_arg=--robot="cb3" --test_arg=robotip="127.0.0.1"
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", help="robot generation to test against", choices=["e-series", "cb3"], default="e-series")
    parser.add_argument("--robotip", help="ip address of the robot", default="127.0.0.1")
    args = parser.parse_args()
    unittest.main(argv=["--robot, --robotip"])