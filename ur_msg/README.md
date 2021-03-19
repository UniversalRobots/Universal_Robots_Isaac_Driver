# UR message

This folder contains messages developed specific to the **Universal_Robots_Isaac_Driver**.
The folder contains:

- **dashboard_command** Consist of two messages. One used to send dashboard commands
to the codelet **DashboardClientIsaac**. And one to receive the status of the command.

- **dashboardCommand** Helper functions to translate the dashboard commands from
an enum in capnp format to a C++ enum.

- **speed_slider** Consist of a message to set the actual value of the speedslider.

- **ur_msg_python** The file contains Python functionality to use ur_msg inside
Python application, see documentation for [ur_msg in python](#UR-message-in-Python)
and [DashboardClientIsaac in Python](#DashboardClientIsaac-usage-in-python).

## Messages

The different messages are described here:

### DashboardCommandProto

For documentation on the dashboard server see [e-series](https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/)
and [cb3](https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/)

```capnp
# A message to represent all dashboard commands available
# For documentation about the dashboard server see
#   - https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
#   - https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/
struct DashboardCommandProto
{
  # Enum holding all dashboard commands available.
  enum DashboardCommand
  {
    load                      @0;   # Load program </programs/program.urp>
    play                      @1;   # Play program
    stop                      @2;   # Stop program
    pause                     @3;   # Pause program
    quit                      @4;   # Disconnect client
    shutdown                  @5;   # Shut down robot
    running                   @6;   # Programm running?
    robotmode                 @7;   # Current robot mode
    getLoadedProgram          @8;   # Returns loaded program
    popup                     @9;   # Show popup <message>
    closePopup                @10;  # Close popup
    addToLog                  @11;  # Add message to log <log-message>
    isProgramSaved            @12;  # Is current program saved?
    programState              @13;  # Returns current program state
    polyscopeVersion          @14;  # Polyscope version number
    setOperationalMode        @15;  # Set operational mode <mode>
    getOperationalMode        @16;  # Returns curretn operationalmode
    clearOperationalMode      @17;  # Dashboard server no longer controls operational mode
    powerOn                   @18;  # Power on robot
    powerOff                  @19;  # Power down robot
    brakeRelease              @20;  # Brake release
    safetystatus              @21;  # Current safety status
    unlockProtectiveStop      @22;  # Unlocks protective stop
    closeSafetyPopup          @23;  # Close safety popup
    loadInstallation          @24;  # Load installation </programs/default.installation>
    restartSafety             @25;  # Restart safety
    isInRemoteControl         @26;  # Returns remote control status
    getSerialNumber           @27;  # Returns serial number
    getRobotModel             @28;  # Returns robot model
    safetymode                @29;  # Current safetymode
    setUserRole               @30;  # Setting user role <role>
    getUserRole               @31;  # Get current user role
  }

  # The request/command to send to the dashboardserver.
  dashboardRequest @0: DashboardCommand;

  # Possible argument to the dashboard command can be empty.
  argument @1: Text = "";
}

```

### DashboardCommandStatusProto

```capnp
# A message to represent the status of a dashboard command.
struct DashboardCommandStatusProto {

  # The anwser from the dashboard server.
  anwser @0: Text;

  # Returns whether the command was a succes or not.
  succes @1: Bool;
}
```

### SpeedSliderProto

```capnp
# Message to set the actual value of the speedslider. Values should be between
# 0 and 1. Only set this smaller than 1 if you are using the scaledMultiJointController
# or you know what you're doing. Using this with another controller might lead to
unexpected behaviors.
struct SpeedSliderProto {
  # The speed slider value
  speedSliderValue @0 : Float32;
}
```

## UR message usage

The following documentation shows how to use ur messages in your applications.

### UR message in C++

To use UR messages in C++ applications, we need to use the ur message inside
codelets. UR messages can be used in the same way as regular Isaac messages.
See Isaac documentation on [sending messages](https://docs.nvidia.com/isaac/isaac/doc/tutorials/ping.html#sending-messages)
and [receiving messages](https://docs.nvidia.com/isaac/isaac/doc/tutorials/ping.html#receiving-messages)

### UR message in Python

Since support for Python is in an experimental state in Isaac, support for the ur_msg
in Python is also in an experimental state. Therefore things might change in the
future. Isaac supports interacting with a running application in Python,
which the below examples uses.

To use UR messages in a Python application two functions has been created, which
can be found in *ur_msg_python.py*. The function *create_ur_msg* are used to
create a message which can be published to a specific channel in a Python application.
The function *get_ur_msg* can be used to create a wrapper for a recieved ur_message
in a Python application.

See the following example on how to send and receive UR messages in a Python application.
The examples uses the [drivers subgraph](../ur_robot_driver/apps/ur_eseries_robot.subgraph.json)
from ur_robot_driver.

Start by creating a folder inside `isaac/sdk/packages` to store the codelet. All
the following files, should be created inside this folder.

```bash
$ mkdir ~/isaac/sdk/packages/ur_msg_example
```

Next we need to create a file to store our Python application. Create a file called
`ur_msg_example.py` and copy the content below into it.

```Python
import time
from packages.universal_robots.ur_msg.ur_msg_python import create_ur_msg, get_ur_msg

from packages.pyalice import Application

app = Application(name="Sample ur message")

app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

# Remember to set the robots IP address
ur_driver = app.nodes["ur.universal_robots"]["UniversalRobots"]
ur_driver.config.robot_ip = "192.168.56.1"

app.start()

# Make sure the driver is running
time.sleep(5)

# Publish ur message (SpeedSliderProto)
speed_fraction = 0.5
speed_fraction_msg = create_ur_msg("SpeedSliderProto")
speed_fraction_msg.proto.speedSliderValue = speed_fraction
app.publish("ur.subgraph", "interface", "set_speed_slider", speed_fraction_msg)

# Publish ur message (DashboardCommandProto)
dashboard_msg = create_ur_msg("DashboardCommandProto")
dashboard_msg.proto.dashboardRequest = "popup"
dashboard_msg.proto.argument = "This is a popup message"
app.publish("ur.subgraph", "interface", "dashboard_command", dashboard_msg)

# Sleep before receiving anwser
time.sleep(1)

# Receive ur message (DashboardCommandStatusProto)
dashboard_anwser = app.receive("ur.subgraph", "interface", "dashboard_anwser")
if dashboard_anwser is not None:
    ur_msg = get_ur_msg(dashboard_anwser)
    print(ur_msg.anwser)
    print(ur_msg.success)

app.stop()
```

**NOTE** Remember to update the robot_ip to the IP address on which the Isaac driver
can reach the robot.

Lastly we need to create the build file. Create a file called `BUILD` and copy the
content below into it.

```bazel
load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "ur_msg_example",
    srcs = ["ur_msg_example.py"],
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/universal_robots/ur_robot_driver/apps:ur_eseries_robot_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_cb3_robot_subgraph",
        "//packages/universal_robots/ur_msg:ur_msg_python"
    ],
    modules = [
        "planner",
        "universal_robots",
    ],
    deps = [
        "//packages/pyalice",
    ],
)
```

Now you can run the application inside the `~/isaac/sdk/` folder by typing:

```bash
$ bazel run packages/ur_msg_example:ur_msg_example
```

This should open a popup message on the teach pendant, and you should also see the
speed slider is set to 50%.

## DashboardClientIsaac usage

To use the dashboard command messages created in this folder a codelet named DashboardClientIsaac
has been created to serve as an interface between the dashboard server on the robot
and an Isaac aplication. This is created in order to use the robot in an Isaac application
without interacting with the teach pendant. Documentation on DashboardClientIsaac
codelet can be found [here](../ur_robot_driver/doc/component_api.md#DashboardClientIsaac).

This serves as a documentation on how to use the DashboardclientIsaac in an
application. It comes with some examples on how to use the DashboardClientIsaac
in C++ applications and in Python applications.

See the [message documentation](#DashboardCommandProto), for documentation on all
the dashboard commands available.

### DashboardClientIsaac usage in C++

Since the DashboardClientIsaac is a C++ codetlet it can be used as any other codelet
in your application. See Isaac documentation on [developing codelets](https://docs.nvidia.com/isaac/isaac/doc/tutorials/ping.html#)
and on [understanding codelets](https://docs.nvidia.com/isaac/isaac/doc/engine/components.html).

Communicating with the DashboardClientIsaac codelet inside C++ codelets. can be
done in the same way as communications between other C++ codelets. See Isaac documentation
on [sending messages](https://docs.nvidia.com/isaac/isaac/doc/tutorials/ping.html#sending-messages)
and [receiving messages](https://docs.nvidia.com/isaac/isaac/doc/tutorials/ping.html#receiving-messages).

The below example shows how to create a small codelet that can communicate with
the DashboardClientIsaac codelet. It will send a dashboard command and then afterwards
receive the anwser from the command and then the codelet will report success. This
example makes use of the [drivers subgraph](../ur_robot_driver/apps/ur_eseries_robot.subgraph.json)
from ur_robot_driver.

Start by creating a folder inside `isaac/sdk/packages` to store the codelet. All
the following files, should be created inside this folder.

```bash
$ mkdir ~/isaac/sdk/packages/dashboard_example
```

Next we want to create the header file for the codelet. Create a file called `DashboardExample.hpp`
and copy the content below into it.

```c++
#pragma once
#include "engine/alice/alice_codelet.hpp"
#include "packages/universal_robots/ur_msg/dashboard_command.capnp.h"
#include "packages/universal_robots/ur_msg/dashboardCommand.hpp"

namespace isaac
{
namespace dashboard_example
{

class DashboardExample : public isaac::alice::Codelet {
public:
  void start() override;
  void tick() override;

  // Publish the dashboard command
  ISAAC_PROTO_TX(DashboardCommandProto, dashboard_command);

  // Channel to receive the anwser from the dashbordserver
  ISAAC_PROTO_RX(DashboardCommandStatusProto, dashboard_anwser);

  bool command_published_;
  std::optional<int64_t> message_time_;
};
}
}
ISAAC_ALICE_REGISTER_CODELET(isaac::dashboard_example::DashboardExample);
```

Next we want to create the cpp file for the codelet. Create a file called `DashboardExample.cpp`
and copy the content below into it.

```c++
#include "DashboardExample.hpp"

namespace isaac
{
namespace dashboard_example
{
void DashboardExample::start()
{
  command_published_ = false;
  tickPeriodically();
}

void DashboardExample::tick()
{
  if (!command_published_ && getTickTime() > 5.0)
  {
    // Make sure that we dont collect and old message as the anwser
    if (rx_dashboard_anwser().available())
    {
      message_time_ = rx_dashboard_anwser().acqtime();
    }
    // Publish popup command
    auto proto_sender = tx_dashboard_command().initProto();
    proto_sender.setDashboardRequest(dashboard_command::ToProto(dashboard_command::DashboardCommand::popup));
    proto_sender.setArgument("This message is added from an Isaac application");

    tx_dashboard_command().publish();
    command_published_ = true;
  }

  // Receive anwser from dashboard server
  if (command_published_ && rx_dashboard_anwser().available())
  {
    if (!message_time_ || rx_dashboard_anwser().acqtime() > *message_time_)
    {
      auto proto = rx_dashboard_anwser().getProto();
      LOG_INFO("Received anwser: %s", proto.getAnwser().cStr());
      LOG_INFO("The command was a succes? %i", proto.getSuccess());
      reportSuccess();
    }
  }
}

}
}
```

Next we need to create our application JSON file. Create a file called `dashboard_example.app.json`
and copy the content below into it.

```JSON
{
  "name": "dashboard_example",
  "modules": [
    "//packages/dashboard_example:dashboard_example_components"
  ],
  "graph": {
    "nodes": [
      {
        "name": "ur_driver",
        "subgraph": "packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json"
      },
      {
        "name": "dashboard_example",
        "components": [
          {
            "name": "MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "DashboardExample",
            "type": "isaac::dashboard_example::DashboardExample"
          }
        ]
      }
    ],
    "edges": [
      {
        "source": "ur_driver.subgraph/interface/dashboard_anwser",
        "target": "dashboard_example/DashboardExample/dashboard_anwser"
      },
      {
        "source": "dashboard_example/DashboardExample/dashboard_command",
        "target": "ur_driver.subgraph/interface/dashboard_command"
      }
    ]
  },
  "config": {
    "dashboard_example": {
      "DashboardExample": {
        "tick_period": "10Hz"
      }
    },
    "ur_driver.universal_robots": {
      "UniversalRobots": {
        "robot_ip": "192.168.56.1"
      }
    }
  }
}
```

**NOTE** Remember to update the robot_ip to the IP address on which the Isaac driver
can reach the robot

Lastly we need to create the build file. Create a file called `BUILD` and copy the
content below into it.

```bazel
load("//bzl:module.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "dashboard_example_components",
    srcs = [
        "DashboardExample.cpp"
    ],
    hdrs = [
        "DashboardExample.hpp"
    ],
    deps = [
        "//packages/universal_robots/ur_msg:dashboard_command",
        "//packages/universal_robots/ur_msg:dashboard_command_proto",
    ]
)

isaac_app(
    name = "dashboard_example",
    data = [
        "//packages/universal_robots/ur_robot_driver/apps:ur_eseries_robot_subgraph"
    ],
    modules = [
       "//packages/dashboard_example:dashboard_example_components",
    ]
)
```

Now you can run the application inside the `~/isaac/sdk/` folder by typing:

```bash
$ bazel run packages/dashboard_example:dashboard_example
```

You should now see a popup message on the teach pendant with the message `This message
is added from an Isaac application`.

This is just a quick example on how to use the DashboardClientIsaac, but serves
as inspiration on how to use DashboardClientIsaac to interact with the robot
without interacting with the teach pendant.

### DashboardClientIsaac usage in python

Since support for Python is in an experimental state in Isaac, support for the dashboardClientIsaac
in Python is also in an experimental state. Therefore things might change in the
future. Isaac supports interacting with a running application in Python,
which the below examples uses.

To use the DashboardClientIsaac Python and to be able to send and receive dashboard
commands, some additional functionality has been created, which can be found in *ur_msg_python.py*.
The function *do_dashboard_command* can be used to execute a dashboard command and
returns the anwser from the command, or None if no anwser was received.

See the following examples on how to communicate with the dashboard server in a running
application in Python. The examples uses the [drivers subgraph](../ur_robot_driver/apps/ur_eseries_robot.subgraph.json)
from ur_robot_driver.

If you haven't created the dashboard_example folder, go ahead and create it.

```bash
$ mkdir ~/isaac/sdk/packages/dashboard_example
```

Next we need to create a file to store our Python application. Create a file called
`py_dashboard_example.py` and copy the content below into it.

```Python
import time
from packages.pyalice import Application
from packages.universal_robots.ur_msg.ur_msg_python import do_dashboard_command

app = Application(name="Sample dashboard client")

app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

# Remember to set the robots IP address
ur_driver = app.nodes["ur.universal_robots"]["UniversalRobots"]
ur_driver.config.robot_ip = "192.168.56.1"

app.start()

# Make sure the driver is running
time.sleep(5)

anwser = do_dashboard_command(app, "popup", arguments="This message is added from an Isaac application")
if anwser is not None:
    print(anwser.anwser)
    print(anwser.success)

# We can also power on the robot
anwser = do_dashboard_command(app, "powerOn")
if anwser is not None:
    print(anwser.anwser)
    print(anwser.success)

app.stop()
```

**NOTE** Remember to update the robot_ip to the IP address on which the Isaac driver
can reach the robot.

If you haven't already created the build file. Go ahead and create a file called
`BUILD` and copy the content below into it else just copy the content below into
the already created build file.

```bazel
load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "py_dashboard_example",
    srcs = ["py_dashboard_example.py"],
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/universal_robots/ur_robot_driver/apps:ur_eseries_robot_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_cb3_robot_subgraph",
        "//packages/universal_robots/ur_msg:ur_msg_python"
    ],
    modules = [
        "planner",
        "universal_robots",
    ],
    deps = [
        "//packages/pyalice",
    ],
)
```

Now you can run the application inside the `~/isaac/sdk/` folder by typing:

```bash
$ bazel run packages/dashboard_example:py_dashboard_example
```

You should now see a popup message on the teach pendant with the message `This message
is added from an Isaac application`.

The below example can be used to power on a robot and playing the currently loaded
program right after starting your application. You can copy the content below into
`py_dashboard_example.py` to use this example.

```Python
import time
from packages.pyalice import Application
from packages.universal_robots.ur_msg.ur_msg_python import do_dashboard_command

app = Application(name="Sample dashboard client")

app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")

# Remember to set the robots IP address
ur_driver = app.nodes["ur.universal_robots"]["UniversalRobots"]
ur_driver.config.robot_ip = "192.168.56.1"

app.start()

# Make sure the driver is running
time.sleep(5)

# The following can be used make sure the robot is powered on.
anwser = do_dashboard_command(app, "powerOn")
if anwser is not None:
    print(anwser.anwser)
    print(anwser.success)

robot_mode = do_dashboard_command(app, "robotmode")
timeout = 10.0
wait_time = 0.0
while wait_time < timeout:
    robot_mode = do_dashboard_command(app, "robotmode")
    if robot_mode is not None:
        if robot_mode.anwser == "Robotmode: IDLE" or robot_mode.anwser == "Robotmode: RUNNING":
            break

    time.sleep(0.1)
    wait_time += 0.1

# Now we can brakerelease the robot
anwser = do_dashboard_command(app, "brakeRelease")
if anwser is not None:
    print(anwser.anwser)
    print(anwser.success)

robot_mode = do_dashboard_command(app, "robotmode")
timeout = 10.0
wait_time = 0.0
while wait_time < timeout:
    robot_mode = do_dashboard_command(app, "robotmode")
    if robot_mode is not None and robot_mode.anwser == "Robotmode: RUNNING":
        break

    time.sleep(0.1)
    wait_time += 0.1

time.sleep(1)

# Now we can play the currently loaded robot program
anwser = do_dashboard_command(app, "play")
if anwser is not None:
    print(anwser.anwser)
    print(anwser.success)

app.stop()
```

**NOTE** Remember to update the robot_ip to the IP address on which the Isaac driver
can reach the robot.

Now you can run the application inside the `~/isaac/sdk/` folder by typing:

```bash
$ bazel run packages/dashboard_example:py_dashboard_example
```

After running the above example the robot should be powered on and the currently
loaded program should start playing.

This is just a quick example on how to use the DashboardClientIsaac, but serves
as inspiration on how to use DashboardClientIsaac to interact with the robot
without interacting with the teach pendant.
