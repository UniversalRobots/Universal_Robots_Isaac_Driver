import time
import re
from .ur_msg import create_ur_msg, get_ur_msg


def wait_for_dc_mode(app, mode, state, timeout=10, node="ur.subgraph", component="interface"):
    """Waits for a specific dashboard mode, could e.g. be used to wait for robot mode is running."""
    wait_time = 0.0
    while wait_time < timeout:
        response = do_dashboard_command(app, mode, node=node, component=component)
        if re.match(".*" + state.strip(), response.anwser):
            break
        time.sleep(0.25)
        wait_time += 0.25

    assert wait_time < timeout, "failed to reach target mode, expected mode: {} current mode: {}".format(mode, response.anwser)

def do_dashboard_command(app, command, arguments="", node="ur.subgraph", component="interface", timeout=5):
    """Execute a dashboard command and waits for the result. Raises exception if no result could be received."""
    dashboard_msg = create_ur_msg("DashboardCommandProto")
    dashboard_msg.proto.dashboardRequest = command
    dashboard_msg.proto.argument = arguments
    cur_time = app.clock.time * 1000000000 # Turn into nano seconds
    app.publish(node, component, "dashboard_command", dashboard_msg)

    # wait for a new message
    wait_time = 0.0
    msg = None
    while wait_time < timeout:
        msg = app.receive(node, component, "dashboard_anwser")
        if msg is not None and cur_time < msg.pubtime:
            break
        time.sleep(0.1)
        wait_time += 0.1

    if msg is None:
        raise Exception("Failed to receive anwser from dashboard server")

    dc_response = get_ur_msg(msg)

    return dc_response

def wait_for_new_message(app, node, component, channel, timeout=2):
    """Waiting for a new message on a specific channel, returns none if no new message was received within the timeout."""
    wait_time = 0.0
    cur_time = app.clock.time * 1000000000 # Turn into nano seconds
    msg = None
    while wait_time < timeout:
        msg = app.receive(node, component, channel)
        if msg is not None:
            if cur_time < msg.pubtime:
                break
        time.sleep(0.1)
        wait_time += 0.1

    return msg