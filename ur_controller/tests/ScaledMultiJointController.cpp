// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include <vector>

#include "packages/composite/gems/serializer.hpp"
#include "packages/map/KinematicTree.hpp"
#include "gtest/gtest.h"
// In order to be able to test the private functions of ScaledMultiJointController and access private attributes
#define private public
#include "packages/universal_robots/ur_controller/ScaledMultiJointController.hpp"

namespace isaac
{
namespace ur_controller
{
std::vector<std::string> joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                         "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

class PublishPlan : public isaac::alice::Codelet
{
public:
  void start() override
  {
    initQuinticSchema();
    tickPeriodically();
  }
  void tick() override
  {
    if (get_send_trajectory())
    {
      set_send_trajectory(false);
      if (schema_name_ == "linear")
      {
        auto proto_builder = tx_trajectory().initProto();
        Timeseries<Vector<double, 7>, double> plan;
        Vector<double, 7> state;
        state << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        plan.tryPush(1.0, state);
        plan.tryPush(2.0, state);
        plan.tryPush(3.0, state);
        plan.tryPush(4.0, state);
        serializer_.serialize(plan, composite::Quantity::Scalar("time", composite::Measure::kTime), proto_builder,
                              tx_trajectory().buffers());
        tx_trajectory().publish();
      }
      else if (schema_name_ == "cubic")
      {
        set_send_trajectory(false);
        auto proto_builder = tx_trajectory().initProto();
        Timeseries<Vector<double, 13>, double> plan;
        Vector<double, 13> state;
        state << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        plan.tryPush(1.0, state);
        plan.tryPush(2.0, state);
        plan.tryPush(3.0, state);
        plan.tryPush(4.0, state);
        serializer_.serialize(plan, composite::Quantity::Scalar("time", composite::Measure::kTime), proto_builder,
                              tx_trajectory().buffers());
        tx_trajectory().publish();
      }
      else if (schema_name_ == "quintic")
      {
        set_send_trajectory(false);
        auto proto_builder = tx_trajectory().initProto();
        Timeseries<Vector<double, 19>, double> plan;
        Vector<double, 19> state;
        state << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        plan.tryPush(1.0, state);
        plan.tryPush(2.0, state);
        plan.tryPush(3.0, state);
        plan.tryPush(4.0, state);
        serializer_.serialize(plan, composite::Quantity::Scalar("time", composite::Measure::kTime), proto_builder,
                              tx_trajectory().buffers());
        tx_trajectory().publish();
      }
      else if (schema_name_ == "invalid")
      {
        set_send_trajectory(false);
        auto proto_builder = tx_trajectory().initProto();
        Timeseries<Vector<double, 16>, double> plan;
        Vector<double, 16> state;
        state << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        plan.tryPush(1.0, state);
        plan.tryPush(2.0, state);
        plan.tryPush(3.0, state);
        plan.tryPush(4.0, state);
        serializer_.serialize(plan, composite::Quantity::Scalar("time", composite::Measure::kTime), proto_builder,
                              tx_trajectory().buffers());
        tx_trajectory().publish();
      }
    }
  }
  void initLinearSchema()
  {
    std::vector<composite::Quantity> quantities;
    quantities.push_back(composite::Quantity::Scalar("time", composite::Measure::kTime));
    for (unsigned int i = 0; i < joint_names.size(); ++i)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kPosition));
    }

    schema_ = composite::Schema(std::move(quantities));
    serializer_.setSchema(schema_);
    schema_name_ = "linear";
  }
  void initCubcicSchema()
  {
    std::vector<composite::Quantity> quantities;
    quantities.push_back(composite::Quantity::Scalar("time", composite::Measure::kTime));
    for (unsigned int i = 0; i < joint_names.size(); ++i)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kPosition));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kSpeed));
    }

    schema_ = composite::Schema(std::move(quantities));
    serializer_.setSchema(schema_);
    schema_name_ = "cubic";
  }
  void initQuinticSchema()
  {
    std::vector<composite::Quantity> quantities;
    quantities.push_back(composite::Quantity::Scalar("time", composite::Measure::kTime));
    for (unsigned int i = 0; i < joint_names.size(); ++i)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kPosition));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kSpeed));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kAcceleration));
    }
    schema_ = composite::Schema(std::move(quantities));
    serializer_.setSchema(schema_);
    schema_name_ = "quintic";
  }
  void initInvalidSchema()
  {
    std::vector<composite::Quantity> quantities;
    quantities.push_back(composite::Quantity::Scalar("time", composite::Measure::kTime));
    for (unsigned int i = 0; i < joint_names.size() - 1; ++i)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kPosition));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kSpeed));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kAcceleration));
    }
    schema_ = composite::Schema(std::move(quantities));
    serializer_.setSchema(schema_);
    schema_name_ = "invalid";
  }
  ISAAC_PROTO_TX(CompositeProto, trajectory);

  // Used to trigger when to send new trajecotry
  ISAAC_PARAM(bool, send_trajectory, false);

  composite::Schema schema_;
  composite::Serializer serializer_;
  std::string schema_name_ = "";
};

class PublishRobotState : public isaac::alice::Codelet
{
  void start() override
  {
    initStateSchema();
    tickPeriodically();
  }
  void tick() override
  {
    auto proto_builder = tx_robot_state().initProto();
    composite::WriteSchema(state_schema_, proto_builder);

    Tensor1d state_data(13);
    for (unsigned int i = 0; i < 12; ++i)
    {
      state_data(i) = 1;
    }
    state_data(12) = get_speed_fraction();

    ToProto(std::move(state_data), proto_builder.initValues(), tx_robot_state().buffers());
    tx_robot_state().publish();
  }
  void initStateSchema()
  {
    std::vector<composite::Quantity> quantities;
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kPosition));
      quantities.push_back(composite::Quantity::Scalar(joint_names[i], composite::Measure::kSpeed));
    }
    quantities.push_back(composite::Quantity::Scalar("speed_fraction", composite::Measure::kNone));
    state_schema_ = composite::Schema(std::move(quantities));
  }

  ISAAC_PROTO_TX(CompositeProto, robot_state);
  // Speed fraction
  ISAAC_PARAM(double, speed_fraction, 1.0);

  composite::Schema state_schema_;
};

class ReceiveJointCommand : public isaac::alice::Codelet
{
  void start() override
  {
    initCommandParser(get_control_mode());
    tickPeriodically();
  }
  void tick() override
  {
    if (rx_joint_command().available())
    {
      if (control_mode_ != get_control_mode())
      {
        initCommandParser(get_control_mode());
      }
      // Make sure we can parse command received from controller based upon control mode
      VectorXd command(6);
      EXPECT_TRUE(command_parser_.parse(rx_joint_command().getProto(), rx_joint_command().buffers(), command));
    }
  }
  void initCommandParser(const ArmControlMode mode)
  {
    control_mode_ = mode;
    switch (mode)
    {
      case kJointPosition:
        command_parser_.requestSchema(composite::Schema(joint_names, composite::Measure::kPosition));
        break;
      case kJointSpeed:
        command_parser_.requestSchema(composite::Schema(joint_names, composite::Measure::kSpeed));
        break;
      default:
        command_parser_.requestSchema({});
        return;
    }
  }

  ISAAC_PROTO_RX(CompositeProto, joint_command);
  // Control mode
  ISAAC_PARAM(ArmControlMode, control_mode, kJointPosition);

  ArmControlMode control_mode_;
  composite::Parser command_parser_;
};

TEST(ScaledMultiJointController, InvalidKinematicTreeComponentTest)
{
  alice::Application first_app;

  // Setup nodes
  auto node = first_app.createMessageNode("test");
  auto controller = node->addComponent<ScaledMultiJointController>();

  // Setup configuration
  controller->async_set_tick_period("100Hz");

  first_app.enableStopOnTimeout(0.2);
  first_app.runBlocking();

  std::string status_msg = controller->getStatusMessage();
  std::string expected_msg = "KinematicTree node is not specified.";
  EXPECT_EQ(controller->getStatus(), alice::Status::FAILURE);
  EXPECT_STREQ(status_msg.c_str(), expected_msg.c_str());

  alice::Application second_app;
  // Setup nodes
  node = second_app.createMessageNode("test");
  controller = node->addComponent<ScaledMultiJointController>();

  // Setup configuration
  controller->async_set_kinematic_tree("apps/assets/kinematic_trees/ur10.kinematic.json");
  controller->async_set_tick_period("100Hz");

  second_app.enableStopOnTimeout(0.2);
  second_app.runBlocking();

  status_msg = controller->getStatusMessage();
  expected_msg = "Node apps/assets/kinematic_trees/ur10.kinematic.json does not contain KinematicTree component.";
  EXPECT_EQ(controller->getStatus(), alice::Status::FAILURE);
  EXPECT_STREQ(status_msg.c_str(), expected_msg.c_str());
}

TEST(ScaledMultiJointController, ToleranceTest)
{
  alice::Application app;
  // Setup nodes
  auto controller_node = app.createMessageNode("controller");
  auto controller = controller_node->addComponent<ScaledMultiJointController>();

  auto kinematic_tree_node = app.createMessageNode("kinematic");
  auto kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");

  app.runAsync();
  Sleep(500000000);  // give time to start application

  // No tolerance configured, it should always return true
  EXPECT_TRUE(controller->insideTolerance(0.2));
  EXPECT_TRUE(controller->insideTolerance(10));

  controller->async_set_tolerance(0.1);
  Sleep(500000000);  // Give time to set the tolerance

  // Inside tolerance
  EXPECT_TRUE(controller->insideTolerance(0.05));
  // At tolerance
  EXPECT_TRUE(controller->insideTolerance(0.1));
  // Outside tolerance
  EXPECT_FALSE(controller->insideTolerance(0.2));

  app.stopBlocking();
}

TEST(ScaledMultiJointController, InitTrajectoryTest)
{
  alice::Application app;

  // Setup nodes
  auto controller_node = app.createMessageNode("controller");
  auto controller = controller_node->addComponent<ScaledMultiJointController>();

  auto kinematic_tree_node = app.createMessageNode("kinematic");
  auto kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");

  app.runAsync();
  Sleep(500000000);  // give time to start application

  std::vector<Timeseries<VectorXd, double> > trajectory(joint_names.size());

  VectorXd start_state(1);
  VectorXd mid_state(1);
  VectorXd end_state(1);
  start_state << 1.0;
  mid_state << 1.5;
  end_state << 2.0;

  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    trajectory[i].tryPush(1.0, start_state);
    trajectory[i].tryPush(2.0, mid_state);
    trajectory[i].tryPush(3.0, end_state);
  }

  controller->initTrajectory(trajectory);
  // We expect an active goal and 0 successful joints after init trajectory
  unsigned int count = std::accumulate(controller->successful_joints_.begin(), controller->successful_joints_.end(),
                                       static_cast<std::size_t>(0));
  EXPECT_TRUE(controller->active_goal_ == true);
  EXPECT_TRUE(count == 0);
  // Make sure endtime matches end time trajectory for each joint
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    EXPECT_TRUE(controller->cur_traj_[i].endTime() == 3.0);
  }
  // We expect active_pid_ to be false, because we havn't configured any gains
  EXPECT_TRUE(controller->active_pid_ == false);

  nlohmann::json gains = {
    { "shoulder_pan_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } },
    { "shoulder_lift_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } },
    { "elbow_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } },
    { "wrist_1_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } },
    { "wrist_2_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } },
    { "wrist_3_joint", { { "p", 5 }, { "i", 0.05 }, { "d", 0.1 }, { "i_clamp", 1 }, { "velocity_ff", 0.5 } } }
  };
  ;

  controller->async_set_gains(gains);
  Sleep(500000000);  // give time to set configuration
  controller->initTrajectory(trajectory);

  // We now expect active_pid_ to be true
  EXPECT_TRUE(controller->active_pid_ == true);

  // Test that ff_term is set correctly
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    LOG_INFO("ff_term %f", controller->ff_term_[i]);
    EXPECT_TRUE(controller->ff_term_[i] == 0.5);
  }

  app.stopBlocking();
}

TEST(ScaledMultiJointController, ControlPeriodTest)
{
  alice::Application app;

  // Setup nodes
  auto controller_node = app.createMessageNode("controller");
  auto controller = controller_node->addComponent<ScaledMultiJointController>();

  auto kinematic_tree_node = app.createMessageNode("kinematic");
  auto kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  auto planner_node = app.createMessageNode("planner");
  auto planner = planner_node->addComponent<PublishPlan>();

  auto robot_state_node = app.createMessageNode("robotState");
  auto robot_state = robot_state_node->addComponent<PublishRobotState>();

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");

  planner->async_set_tick_period("10Hz");
  planner->async_set_send_trajectory(true);

  robot_state->async_set_tick_period("100Hz");

  // Setup communication
  Connect(planner->tx_trajectory(), controller->rx_plan());
  Connect(robot_state->tx_robot_state(), controller->rx_current_arm_state());

  app.runAsync();
  Sleep(500000000);  // give time to start application

  double expected_period = 0.01;
  double uptime, period;
  double abs = 0.001;

  uptime = controller->controller_uptime_;
  period = controller->period_;

  EXPECT_NEAR(expected_period, period, abs);
  // When the speed isn't scaled we expect controller_uptime and tick_time to be equal
  EXPECT_NEAR(uptime, controller->tick_time_, abs);

  robot_state->async_set_speed_fraction(0.5);

  Sleep(500000000);
  expected_period = 0.01 * 0.5;
  period = controller->period_;
  uptime = controller->controller_uptime_;

  EXPECT_NEAR(expected_period, period, abs);
  EXPECT_LT(uptime, controller->tick_time_);

  robot_state->async_set_speed_fraction(0.7);

  Sleep(500000000);
  expected_period = 0.01 * 0.7;
  period = controller->period_;
  uptime = controller->controller_uptime_;

  EXPECT_NEAR(expected_period, period, abs);
  EXPECT_LT(uptime, controller->tick_time_);

  app.stopBlocking();
}

TEST(ScaledMultiJointController, PlanParserTest)
{
  alice::Application app;

  // Setup nodes
  auto controller_node = app.createMessageNode("controller");
  auto controller = controller_node->addComponent<ScaledMultiJointController>();

  auto kinematic_tree_node = app.createMessageNode("kinematic");
  auto kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  auto planner_node = app.createMessageNode("planner");
  auto planner = planner_node->addComponent<PublishPlan>();

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");

  planner->async_set_tick_period("10Hz");

  // Setup communication
  Connect(planner->tx_trajectory(), controller->rx_plan());

  app.runAsync();
  Sleep(500000000);  // give time to start application

  // Make sure that we can parse messages based upon interpolation scheme.
  controller->async_set_interpolation_scheme(InterpolationScheme::linear_interpolation);
  planner->initLinearSchema();
  planner->async_set_send_trajectory(true);

  Sleep(500000000);  // Wait for message to be received
  EXPECT_TRUE(controller->succesful_plan_parse_);

  controller->async_set_interpolation_scheme(InterpolationScheme::cubic_interpolation);
  planner->initCubcicSchema();
  planner->async_set_send_trajectory(true);

  Sleep(500000000);  // Wait for message to be received
  EXPECT_TRUE(controller->succesful_plan_parse_);

  controller->async_set_interpolation_scheme(InterpolationScheme::quintic_interpolation);
  planner->initQuinticSchema();
  planner->async_set_send_trajectory(true);

  Sleep(500000000);  // Wait for message to be received
  EXPECT_TRUE(controller->succesful_plan_parse_);

  planner->initInvalidSchema();
  planner->async_set_send_trajectory(true);

  Sleep(500000000);  // Wait for message to be received
  EXPECT_FALSE(controller->succesful_plan_parse_);

  app.stopBlocking();
}

TEST(ScaledMultiJointController, jointCommandTest)
{
  // First app publishes joint positions as commands
  alice::Application first_app;

  // Setup nodes
  auto controller_node = first_app.createMessageNode("controller");
  auto controller = controller_node->addComponent<ScaledMultiJointController>();

  auto kinematic_tree_node = first_app.createMessageNode("kinematic");
  auto kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  auto planner_node = first_app.createMessageNode("planner");
  auto planner = planner_node->addComponent<PublishPlan>();

  auto joint_command_node = first_app.createMessageNode("joint_command");
  auto joint_command = joint_command_node->addComponent<ReceiveJointCommand>();

  auto robot_state_node = first_app.createMessageNode("robotState");
  auto robot_state = robot_state_node->addComponent<PublishRobotState>();

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");

  planner->async_set_tick_period("10Hz");
  planner->async_set_send_trajectory(true);

  joint_command->async_set_tick_period("10Hz");

  robot_state->async_set_tick_period("100Hz");

  // Setup communication
  Connect(planner->tx_trajectory(), controller->rx_plan());
  Connect(robot_state->tx_robot_state(), controller->rx_current_arm_state());
  Connect(controller->tx_joint_command(), joint_command->rx_joint_command());

  first_app.enableStopOnTimeout(0.2);
  first_app.runBlocking();

  // Second app publishes joint speeds as commands
  alice::Application second_app;

  // Setup nodes
  controller_node = second_app.createMessageNode("controller");
  controller = controller_node->addComponent<ScaledMultiJointController>();

  kinematic_tree_node = second_app.createMessageNode("kinematic");
  kinematic_tree = kinematic_tree_node->addComponent<map::KinematicTree>("kinematic_tree");

  planner_node = second_app.createMessageNode("planner");
  planner = planner_node->addComponent<PublishPlan>();

  joint_command_node = second_app.createMessageNode("joint_command");
  joint_command = joint_command_node->addComponent<ReceiveJointCommand>();

  robot_state_node = second_app.createMessageNode("robotState");
  robot_state = robot_state_node->addComponent<PublishRobotState>();

  // Setup configuration
  kinematic_tree_node->start_order = -100;
  kinematic_tree->async_set_kinematic_file("apps/assets/kinematic_trees/ur10.kinematic.json");

  controller->async_set_tick_period("100Hz");
  controller->async_set_kinematic_tree("kinematic");
  controller->async_set_control_mode(ArmControlMode::kJointSpeed);

  planner->async_set_tick_period("10Hz");
  planner->async_set_send_trajectory(true);

  joint_command->async_set_tick_period("10Hz");
  joint_command->async_set_control_mode(ArmControlMode::kJointSpeed);

  robot_state->async_set_tick_period("100Hz");

  // Setup communication
  Connect(planner->tx_trajectory(), controller->rx_plan());
  Connect(robot_state->tx_robot_state(), controller->rx_current_arm_state());
  Connect(controller->tx_joint_command(), joint_command->rx_joint_command());

  second_app.enableStopOnTimeout(0.2);
  second_app.runBlocking();
}

}  // namespace ur_controller
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ur_controller::PublishPlan);
ISAAC_ALICE_REGISTER_CODELET(isaac::ur_controller::PublishRobotState);
ISAAC_ALICE_REGISTER_CODELET(isaac::ur_controller::ReceiveJointCommand);