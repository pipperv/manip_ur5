/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2/convert.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_demo");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // Gripper publisher setup
  // ros::Publisher gripper = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
  // ros::Rate loop_rate(10);

  // trajectory_msgs::JointTrajectory open_jt;

  // open_jt.joint_names.push_back("joint1");
  // open_jt.joint_names.push_back("joint2");

  // trajectory_msgs::JointTrajectoryPoint open_jtp;

  // open_jtp.positions.push_back(1.0);
  // open_jtp.positions.push_back(-1.0);

  // open_jt.points.push_back(open_jtp);

  // gripper.publish(open_jt);
  // ros::spinOnce();

  ros::Publisher gripper = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",1);
  ros::Rate loop_rate(10);

  // Open Joint Trajectory
  trajectory_msgs::JointTrajectory open_traj;
  open_traj.header.frame_id = "base_link";

  open_traj.joint_names.resize(2);
  open_traj.joint_names[0] ="joint1";
  open_traj.joint_names[1] ="joint2";

  open_traj.points.resize(1);
  open_traj.points[0].positions.resize(2);
  open_traj.points[0].time_from_start = ros::Duration(1);

  open_traj.points[0].positions[0] = 1.0;
  open_traj.points[0].positions[1] = -1.0;

  // Open Joint Trajectory
  trajectory_msgs::JointTrajectory closed_traj;
  closed_traj.header.frame_id = "base_link";

  closed_traj.joint_names.resize(2);
  closed_traj.joint_names[0] ="joint1";
  closed_traj.joint_names[1] ="joint2";

  closed_traj.points.resize(1);
  closed_traj.points[0].positions.resize(2);
  closed_traj.points[0].time_from_start = ros::Duration(1);

  closed_traj.points[0].positions[0] = 0.1;
  closed_traj.points[0].positions[1] = -0.1;

  // TO OPEN GRIPPER
  // ros::Duration(1).sleep();
  // gripper.publish(open_traj);

  // TO CLOSE GRIPPER
  // ros::Duration(1).sleep();
  // gripper.publish(closed_traj);


  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group_interface.setEndEffectorLink("tool0");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.0;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("ur5_demo", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("ur5_demo", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("ur5_demo", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  // TABLE
  moveit_msgs::CollisionObject table;
  table.header.frame_id = move_group_interface.getPlanningFrame();
  // OBJ
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  table.id = "table";
  collision_object.id = "box";

  std::vector<std::string> object_ids;

  object_ids.push_back(table.id);
  object_ids.push_back(collision_object.id);

  move_group_interface.detachObject(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // TABLE DEFINITIONS
  // Define a table to add to the world.
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 3.0;
  table_primitive.dimensions[1] = 3.0;
  table_primitive.dimensions[2] = 0.2;

  // Define a pose for the table (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.1;

  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  // OBJ DEFINITIONS
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = 0.055;
  box_primitive.dimensions[1] = 0.055;
  box_primitive.dimensions[2] = 0.055;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.8;
  box_pose.position.y = 0.3;
  box_pose.position.z = 0.0275;

  collision_object.primitives.push_back(box_primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(table);
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
 
  //Open Gripper before planning
  ros::Duration(1).sleep();
  gripper.publish(open_traj);

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO_NAMED("ur5_demo", "Press 'next' in the RvizVisualToolsGui window to start the demo");
  boost::shared_ptr<sensor_msgs::Joy const> sharedGuiInput;
  sensor_msgs::Joy gui_input;
  sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
  if(sharedGuiInput != NULL){
    gui_input = *sharedGuiInput;
  }

  if(gui_input.buttons[4]){
    ros::shutdown();
    return 0;
  }

  // Planning to 4 Pose goal & Visualizing plans
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());

  move_group_interface.setNumPlanningAttempts(10);

  geometry_msgs::Pose target_pose;

  double roll;
  double pitch;
  double yaw;

  node_handle.getParam("/ur5_demo/roll", roll);
  node_handle.getParam("/ur5_demo/pitch", pitch);
  node_handle.getParam("/ur5_demo/yaw", yaw);

  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  quaternion_tf2 = quaternion_tf2.normalize();

  target_pose.orientation.x = quaternion_tf2.x();
  target_pose.orientation.y = quaternion_tf2.y();
  target_pose.orientation.z = quaternion_tf2.z();
  target_pose.orientation.w = quaternion_tf2.w();
  target_pose.position.x = 0.8;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.3;
  move_group_interface.setStartState(start_state);
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;

  move_group_interface.plan(my_plan1);
  visual_tools.publishAxisLabeled(target_pose, "pose_1");
  visual_tools.publishTrajectoryLine(my_plan1.trajectory_, joint_model_group);

  target_pose.position.z -= 0.08;
  std::vector<double> joints1 = my_plan1.trajectory_.joint_trajectory.points.back().positions;
  start_state.setJointGroupPositions(joint_model_group, joints1);
  move_group_interface.setStartState(start_state);
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  move_group_interface.plan(my_plan2);
  visual_tools.publishAxisLabeled(target_pose, "pose_2");
  visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);

  target_pose.position.z += 0.08;
  std::vector<double> joints2 = my_plan2.trajectory_.joint_trajectory.points.back().positions;
  start_state.setJointGroupPositions(joint_model_group, joints2);
  move_group_interface.setStartState(start_state);
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

  move_group_interface.plan(my_plan3);
  visual_tools.publishAxisLabeled(target_pose, "pose_3");
  visual_tools.publishTrajectoryLine(my_plan3.trajectory_, joint_model_group);

  target_pose.position.y -= 0.5;
  std::vector<double> joints3 = my_plan3.trajectory_.joint_trajectory.points.back().positions;
  start_state.setJointGroupPositions(joint_model_group, joints3);
  move_group_interface.setStartState(start_state);
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

  move_group_interface.plan(my_plan4);
  visual_tools.publishAxisLabeled(target_pose, "pose_4");
  visual_tools.publishTrajectoryLine(my_plan4.trajectory_, joint_model_group);

  visual_tools.trigger();

  ROS_INFO_NAMED("ur5_demo", "Press 'next' to execute planning");
  sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
  if(sharedGuiInput != NULL){
    gui_input = *sharedGuiInput;
  }

  if(gui_input.buttons[4]){
    ros::shutdown();
    return 0;
  }

  // Executing Paths
  // ^^^^^^^^^^^^^^^^^

  move_group_interface.execute(my_plan1);
  ros::Duration(1).sleep();
  move_group_interface.execute(my_plan2);
  //Close Gripper after going grasp pose
  move_group_interface.attachObject(collision_object.id);
  gripper.publish(closed_traj);
  ros::Duration(1).sleep();
  move_group_interface.execute(my_plan3);
  ros::Duration(1).sleep();
  move_group_interface.execute(my_plan4);

  ros::shutdown();
  return 0;
}
