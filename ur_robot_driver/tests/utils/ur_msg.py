import glob
import capnp
from packages.pyalice import Message

def create_ur_msg_dictornary():
    """Load all the capnp'n'proto schemata in the ur_msg folder. The function will glob through all the
    files with "*.capnp" extension name."""
    capnp_dict = {}
    ur_capnp_files = glob.glob("packages/universal_robots/ur_msg/*.capnp")
    for capnp_file in ur_capnp_files:
        module = capnp.load(capnp_file)
        for name, obj in module.__dict__.items():
            if obj.__class__.__name__ == "_StructModule":
                capnp_dict[name] = obj

    return capnp_dict

def ur_capnp_schema_type_id_dict():
    """Creates a dictionary which maps Capn'proto type ids to class schemata."""
    capnp_files = glob.glob("packages/universal_robots/ur_msg/*.capnp")

    result = {}
    for capnp_f in capnp_files:
        module = capnp.load(capnp_f)
        for name, obj in module.__dict__.items():
            if obj.__class__.__name__ == "_StructModule":
                assert name not in result
                result[obj.schema.node.id] = obj

    return result

UR_CAPNP_DICT = create_ur_msg_dictornary()
UR_CAPNP_TYPE_ID_DICT = ur_capnp_schema_type_id_dict()

def create_ur_msg(msg_type):
    """Creates a proto message for populating and publishing from specified proto name."""
    msg = Message.MessageBuilder()
    msg.proto = UR_CAPNP_DICT[msg_type].new_message()
    return msg

def get_ur_msg(msg):
    """Creates a wrapper for received message (Read-Only)."""
    data = msg._message.get_capn_proto_bytes()
    if data:
        msg._proto = UR_CAPNP_TYPE_ID_DICT[msg.type_id].from_bytes(data)
    return msg._proto