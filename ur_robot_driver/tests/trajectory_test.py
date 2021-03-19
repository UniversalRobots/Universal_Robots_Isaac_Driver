import unittest
import numpy as np
import time
import argparse
from utils.ur_msg import create_ur_msg
from utils.test_utils import do_dashboard_command, wait_for_new_message, wait_for_dc_mode

from packages.pyalice import Application, Message, Composite

class TrajectoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ip = args.robotip
        robot = args.robot

        cls.app = Application(name="trajectory_test")
        if robot == "e-series":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")
        elif robot == "cb3":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_cb3_robot.subgraph.json", prefix="ur")
        else: # default to eseries
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

        cls.ur_controller = cls.app.nodes["ur.controller"]["ScaledMultiJointController"]
        cls.ur_controller.config.control_mode = "joint position"

        cls.ur_driver = cls.app.nodes["ur.universal_robots"]["UniversalRobots"]
        cls.ur_driver.config.control_mode = "joint position"
        cls.ur_driver.config.robot_ip = ip
        cls.ur_driver.config.headless_mode = True

        planner = cls.app.nodes["ur.local_plan"]["MultiJointLqrPlanner"]
        planner.config.speed_max = [2, 2, 2, 2, 2, 2]
        planner.config.speed_min = [-2,-2,-2,-2,-2,-2]
        planner.config.acceleration_max = [2, 2, 2, 2, 2, 2]
        planner.config.acceleration_min = [-2, -2, -2, -2, -2, -2]

        cls.app.start()

        cls.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        cls.position_parser = [[x, "position", 1] for x in cls.joint_names]

        cls.init_robot(cls)

        cls.joint_values = [[0.0 for i in range(6)]]
        cls.joint_values.append([-1.0 for i in range(6)])

    @classmethod
    def tearDownClass(cls):
        cls.app.stop()

    def init_robot(self):
        """Make sure the robot is booted and ready to receive commands."""
        self.wait_for_driver(self)

        # Make sure ursim is brake released and powered on
        time.sleep(2) # DashboardClient needs to be up and running
        do_dashboard_command(self.app, "brakeRelease")
        wait_for_dc_mode(self.app, "robotmode", "RUNNING")
        do_dashboard_command(self.app, "stop")
        time.sleep(1)

        resend_control_script = Message.create_message_builder("BooleanProto")
        resend_control_script.proto.flag = True
        self.app.publish("ur.subgraph", "interface", "resend_control_script", resend_control_script)
        time.sleep(5)

    def wait_for_driver(self, timeout=30):
        """Make sure the driver is started and publishing robot states."""
        state_msg = wait_for_new_message(self.app, "ur.subgraph", "interface", "arm_state", timeout=timeout)
        if state_msg is None:
            self.fail("Could not receive message from driver. Make sure the driver is actually running")

    def wait_for_trajectory_result(self, timeout=15):
        """Wait for result of executed trajectory."""
        msg = wait_for_new_message(self.app, "ur.controller", "ScaledMultiJointController",
                                             "trajectory_executed_succesfully", timeout=timeout)
        if msg is None:
            self.fail("Could not read trajectory result within timeout {}".format(timeout))

        return msg.proto

    def test_trajectory_joint_position(self):
        """Test robot movement with joint position commands."""
        self.ur_controller.config.control_mode = "joint position"
        self.ur_driver.config.control_mode = "joint position"
        time.sleep(0.1)

        target_pos = np.array(self.joint_values[0], dtype=np.float64)
        target_pos_msg = Composite.create_composite_message(self.position_parser, target_pos)
        self.app.publish("ur.subgraph", "interface", "joint_target", target_pos_msg)
        time.sleep(0.2)

        trajectory_executed_succesfully = self.wait_for_trajectory_result()
        self.assertTrue(trajectory_executed_succesfully.flag == True, "failed to execute trajectory succesfully")

        time.sleep(1)
        arm_state_msg = self.app.receive("ur.subgraph", "interface", "arm_state")
        arm_state = Composite.parse_composite_message(arm_state_msg, self.position_parser)
        self.assertTrue(np.allclose(arm_state, target_pos, rtol=0.01, atol=0.01),
                                    "failed to stay in target position. cur_pos: {}, target pos: {}".format(arm_state, target_pos))

        target_pos = np.array(self.joint_values[1], dtype=np.float64)
        target_pos_msg = Composite.create_composite_message(self.position_parser, target_pos)
        self.app.publish("ur.subgraph", "interface", "joint_target", target_pos_msg)
        time.sleep(0.2)

        trajectory_executed_succesfully = self.wait_for_trajectory_result()
        self.assertTrue(trajectory_executed_succesfully.flag == True, "failed to execute trajectory succesfully")

        time.sleep(1)
        arm_state_msg = self.app.receive("ur.subgraph", "interface", "arm_state")
        arm_state = Composite.parse_composite_message(arm_state_msg, self.position_parser)
        self.assertTrue(np.allclose(arm_state, target_pos, rtol=0.01, atol=0.01),
                                    "failed to stay in target position. cur_pos: {}, target pos: {}".format(arm_state, target_pos))

    def test_trajectory_joint_speed(self):
        """Test robot movement with joint speed commands."""
        self.ur_controller.config.control_mode = "joint speed"
        self.ur_driver.config.control_mode = "joint speed"
        time.sleep(0.1)

        target_pos = np.array(self.joint_values[0], dtype=np.float64)
        target_pos_msg = Composite.create_composite_message(self.position_parser, target_pos)
        self.app.publish("ur.subgraph", "interface", "joint_target", target_pos_msg)
        time.sleep(0.2)

        trajectory_executed_succesfully = self.wait_for_trajectory_result()
        self.assertTrue(trajectory_executed_succesfully.flag == True, "failed to execute trajectory succesfully")

        time.sleep(1)
        recv_msg = self.app.receive("ur.subgraph", "interface", "arm_state")
        cur_pos = Composite.parse_composite_message(recv_msg, self.position_parser)
        self.assertTrue(np.allclose(cur_pos, target_pos, rtol=0.01, atol=0.01),
                                    "failed to stay in target position. cur_pos: {}, target pos: {}".format(cur_pos, target_pos))

        target_pos = np.array(self.joint_values[1], dtype=np.float64)
        target_pos_msg = Composite.create_composite_message(self.position_parser, target_pos)
        self.app.publish("ur.subgraph", "interface", "joint_target", target_pos_msg)
        time.sleep(0.2)

        trajectory_executed_succesfully = self.wait_for_trajectory_result()
        self.assertTrue(trajectory_executed_succesfully.flag == True, "failed to execute trajectory succesfully")

        time.sleep(1)
        recv_msg = self.app.receive("ur.subgraph", "interface", "arm_state")
        cur_pos = Composite.parse_composite_message(recv_msg, self.position_parser)
        self.assertTrue(np.allclose(cur_pos, target_pos, rtol=0.01, atol=0.01),
                                    "failed to stay in target position. cur_pos: {}, target pos: {}".format(cur_pos, target_pos))

if __name__ == "__main__":
    # parse the arguments with --test_arg=--robot="cb3" --test_arg=robotip="127.0.0.1"
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", help="robot generation to test against", choices=["e-series", "cb3"], default="e-series")
    parser.add_argument("--robotip", help="ip address of the robot", default="127.0.0.1")
    args = parser.parse_args()
    unittest.main(argv=["--robot, --robotip"])