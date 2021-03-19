# Controller stopper

A small helper component that stops and restarts Isaac controller based on a boolean
status message. When the status goes to false, all running controllers except a
set of predefined consistent_controller get stopped. If status returns to true
the stopped controller are restarted.

The ControllerStopper component is automatically started within the UniversalRobots
codelet. So no configuration is needed to use this node. The component is explained
further in [component api](../ur_robot_driver/doc/component_api.md#ControllerStopper)
