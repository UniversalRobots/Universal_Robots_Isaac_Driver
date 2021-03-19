import glob
import capnp
import time
from packages.pyalice import Message

def create_ur_msg_dictornary():
    """Load all the capnp'n'proto schemata in the ur_msg folder. The function will glob through all the
    files with "*.capnp" extension name."""
    ur_msg_capnp_dict = {}
    ur_msg_ur_capnp_files = glob.glob("packages/universal_robots/ur_msg/*.capnp")
    for ur_msg_capnp_file in ur_msg_ur_capnp_files:
        module = capnp.load(ur_msg_capnp_file)
        for name, obj in module.__dict__.items():
            if obj.__class__.__name__ == "_StructModule":
                ur_msg_capnp_dict[name] = obj

    return ur_msg_capnp_dict

def ur_capnp_schema_type_id_dict():
    """Creates a dictionary which maps Capn'proto type ids to class schemata."""
    ur_msg_capnp_files = glob.glob("packages/universal_robots/ur_msg/*.capnp")

    result = {}
    for ur_msg_capnp_f in ur_msg_capnp_files:
        module = capnp.load(ur_msg_capnp_f)
        for name, obj in module.__dict__.items():
            if obj.__class__.__name__ == "_StructModule":
                assert name not in result
                result[obj.schema.node.id] = obj

    return result

UR_MSG_CAPNP_DICT = create_ur_msg_dictornary()
UR_MSG_CAPNP_TYPE_ID_DICT = ur_capnp_schema_type_id_dict()

def create_ur_msg(msg_type):
    """Creates a proto message for populating and publishing from specified proto name."""
    msg = Message.MessageBuilder()
    msg.proto = UR_MSG_CAPNP_DICT[msg_type].new_message()
    return msg

def get_ur_msg(msg):
    """Creates a wrapper for received message."""
    data = msg._message.get_capn_proto_bytes()
    if data:
        msg._proto = UR_MSG_CAPNP_TYPE_ID_DICT[msg.type_id].from_bytes(data)
    return msg._proto

def do_dashboard_command(app, command, arguments="", node="ur.subgraph", component="interface", timeout=5):
    """Execute a dashboard command and waits for the result.
    Returns None if no result could be received within timeout."""

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
        app.logger.warning("Failed to receive anwser from DashboardClientIsaac")
        return None

    dc_response = get_ur_msg(msg)
    return dc_response