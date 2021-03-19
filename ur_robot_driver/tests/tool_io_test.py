import unittest
import numpy as np
import time
import argparse
from utils.ur_msg import create_ur_msg
from utils.test_utils import do_dashboard_command, wait_for_new_message, wait_for_dc_mode

from packages.pyalice import Application, Message, Composite

class ToolIoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ip = args.robotip
        robot = args.robot

        cls.app = Application(name="tool_io_test")
        if robot == "e-series":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")
        elif robot == "cb3":
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_cb3_robot.subgraph.json", prefix="ur")
        else: # default to eseries
            cls.app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

        ur_driver = cls.app.nodes["ur.universal_robots"]["UniversalRobots"]
        ur_driver.config.robot_ip = ip
        ur_driver.config.headless_mode = True

        cls.app.start()

        io_names = ["tool_digital_out_0", "tool_digital_out_1"]
        cls.io_parser = [[x, "none", 1] for x in io_names]

    @classmethod
    def tearDownClass(cls):
        cls.app.stop()

    def test_set_tool_io(self):
        """Test setting tool IO values, and check wheter the values has been set."""
        maximum_messages = 5
        io_values = np.array([0, 1], dtype=np.float64)

        io_target_msg = Composite.create_composite_message(self.io_parser, io_values)
        self.app.publish("ur.subgraph", "interface", "io_command", io_target_msg)
        messages = 0
        io_state = None

        while messages < maximum_messages:
            io_state_msg = wait_for_new_message(self.app, "ur.subgraph", "interface", "io_state")
            if io_state_msg is not None:
                io_state = Composite.parse_composite_message(io_state_msg, self.io_parser)
                if((io_state==io_values).all()):
                    break
            messages += 1
        if messages >= maximum_messages:
            self.fail("Could not read desired state after {} messages.".format(maximum_messages))

        self.assertEqual(io_state[0], io_values[0])
        self.assertEqual(io_state[1], io_values[1])

        io_values = np.array([1, 0], dtype=np.float64)

        io_target_msg = Composite.create_composite_message(self.io_parser, io_values)
        self.app.publish("ur.subgraph", "interface", "io_command", io_target_msg)
        messages = 0

        while messages < maximum_messages:
            io_state_msg = wait_for_new_message(self.app, "ur.subgraph", "interface", "io_state")
            if io_state_msg is not None:
                io_state = Composite.parse_composite_message(io_state_msg, self.io_parser)
                if((io_state==io_values).all()):
                    break
            messages +=1
        if messages >= maximum_messages:
            self.fail("Could not read desired state after {} messages.".format(maximum_messages))

        self.assertEqual(io_state[0], io_values[0])
        self.assertEqual(io_state[1], io_values[1])

if __name__ == "__main__":
    # parse the arguments with --test_arg=--robot="cb3" --test_arg=robotip="127.0.0.1"
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", help="robot generation to test against", choices=["e-series", "cb3"], default="e-series")
    parser.add_argument("--robotip", help="ip address of the robot", default="127.0.0.1")
    args = parser.parse_args()
    unittest.main(argv=["--robot, --robotip"])