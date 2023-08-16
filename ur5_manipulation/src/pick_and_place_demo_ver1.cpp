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

/* Author: Felipe Valenzuela */

#include <iostream>
#include <sstream>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>

const double tau = 2 * M_PI;

std::string intToString(int num) {
    std::stringstream ss;
    ss << num;
    return ss.str();
}

bool compareZPosition(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
    return pose1.pose.position.z > pose2.pose.position.z;
}

void arucoTF(tf2_ros::TransformBroadcaster& broadcaster)
{
    // Create a transform object with the specified translation and rotation
    // tf2::Transform transform;
    // transform.setOrigin(translation);
    // transform.setRotation(rotation);
    
    

    // Create a TF broadcaster
    // static tf2_ros::TransformBroadcaster broadcaster;

    // Publish the transform
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "aruco_26_frame";
    tf_msg.transform.translation.x = 0.3;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    // tf_msg.transform = tf2::toMsg(transform);
    // tf2::toMsg(transform, tf_msg.transform);
    broadcaster.sendTransform(tf_msg);
}

moveit_msgs::CollisionObject UR5Base()
{
  moveit_msgs::CollisionObject base;
  base.id = "base";
  base.header.frame_id = "base_link"; // Set the frame ID of the collision object

  // Populate the collision object with shapes and poses
  //METAL RECTANGLE
  shape_msgs::SolidPrimitive metal_rectangle;
  metal_rectangle.type = metal_rectangle.BOX;
  metal_rectangle.dimensions.resize(3);
  metal_rectangle.dimensions[0] = 0.238;
  metal_rectangle.dimensions[1] = 0.412;
  metal_rectangle.dimensions[2] = 0.012;

  geometry_msgs::Pose metal_rectangle_pose;
  tf2::Quaternion quaternion0;
  quaternion0.setRPY(0.0, 0.0, 3*M_PI/4);
  metal_rectangle_pose.orientation.x = quaternion0.x();
  metal_rectangle_pose.orientation.y = quaternion0.y();
  metal_rectangle_pose.orientation.z = quaternion0.z();
  metal_rectangle_pose.orientation.w = quaternion0.w();
  metal_rectangle_pose.position.x = 0.0;
  metal_rectangle_pose.position.y = 0.0;
  metal_rectangle_pose.position.z = -0.006;

  

  //LEG1
  shape_msgs::SolidPrimitive leg1;
  leg1.type = leg1.BOX;
  leg1.dimensions.resize(3);
  leg1.dimensions[0] = 0.9;
  leg1.dimensions[1] = 0.15;
  leg1.dimensions[2] = 0.305;

  geometry_msgs::Pose leg1_pose;
  tf2::Quaternion quaternion1;
  // quaternion1.setRPY(0.0, 0.0, 0.0);
  // leg1_pose.orientation.x = quaternion1.x();
  // leg1_pose.orientation.y = quaternion1.y();
  // leg1_pose.orientation.z = quaternion1.z();
  leg1_pose.orientation.w = 1.0;
  leg1_pose.position.x = 0.1358542930388;
  leg1_pose.position.y = -0.1358542930388;
  leg1_pose.position.z = -0.1645;

  //LEG2
  shape_msgs::SolidPrimitive leg2;
  leg2.type = leg2.BOX;
  leg2.dimensions.resize(3);
  leg2.dimensions[0] = 0.9;
  leg2.dimensions[1] = 0.15;
  leg2.dimensions[2] = 0.305;

  geometry_msgs::Pose leg2_pose;
  tf2::Quaternion quaternion2;
  quaternion2.setRPY(0.0, 0.0, M_PI/2);
  leg2_pose.orientation.x = quaternion2.x();
  leg2_pose.orientation.y = quaternion2.y();
  leg2_pose.orientation.z = quaternion2.z();
  leg2_pose.orientation.w = quaternion2.w();
  leg2_pose.position.x = 0.1358542930388;
  leg2_pose.position.y = -0.1358542930388;
  leg2_pose.position.z = -0.1645;

  base.primitives.push_back(metal_rectangle);
  base.primitives.push_back(leg1);
  base.primitives.push_back(leg2);
  base.primitive_poses.push_back(metal_rectangle_pose);
  base.primitive_poses.push_back(leg1_pose);
  base.primitive_poses.push_back(leg2_pose);
  base.operation = base.ADD;

  // Set other properties if needed, like operation, padding, etc.
  // collision_object.operation = moveit_msgs::CollisionObject::ADD;
  // collision_object.padding = 0.02; // Example padding value in meters

  return base;
}

geometry_msgs::TransformStamped poseStampedToTransformStamped(const geometry_msgs::PoseStamped& poseStamped)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "obj_frame";
    transformStamped.transform.translation.x = poseStamped.pose.position.x;
    transformStamped.transform.translation.y = poseStamped.pose.position.y;
    transformStamped.transform.translation.z = poseStamped.pose.position.z;
    transformStamped.transform.rotation.x = poseStamped.pose.orientation.x;
    transformStamped.transform.rotation.y = poseStamped.pose.orientation.y;
    transformStamped.transform.rotation.z = poseStamped.pose.orientation.z;
    transformStamped.transform.rotation.w = poseStamped.pose.orientation.w;
    return transformStamped;
}

std::vector<geometry_msgs::PoseStamped> generateGraspPose(const geometry_msgs::TransformStamped& transformStamped, const shape_msgs::SolidPrimitive& solid_primitive, const std::string& frame_id)
{
  std::vector<geometry_msgs::PoseStamped> pose_stamped_vector;

  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped real_pose;
  tf2::Quaternion quaternion;
  current_pose.header.frame_id = "base_link";
  for (int i = 0; i <= 2; i++)
  {
    double offset = -solid_primitive.dimensions[i]/2 + std::max(0.0,solid_primitive.dimensions[i]-0.075);
    double gripper_off = 0.225;
    // X
    if (i == 0) {
      //Positive
        current_pose.pose.position.x = (gripper_off + offset);
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.z = 0.0;
        quaternion.setRPY(0.0, -M_PI/2, 0.0);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);
        //Negative
        current_pose.pose.position.x = -(gripper_off + offset);
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.z = 0.0;
        quaternion.setRPY(0.0, M_PI/2, 0.0);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);}
    // Y
    if (i == 0) {
      //Positive
        current_pose.pose.position.y = (gripper_off + offset);
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.z = 0.0;
        quaternion.setRPY(M_PI/2, -M_PI/2, 0.0);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);
        //Negative
        current_pose.pose.position.y = -(gripper_off + offset);
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.z = 0.0;
        quaternion.setRPY(-M_PI/2, -M_PI/2, 0.0);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);}
    // Z
    if (i == 0) {
      //Positive
        current_pose.pose.position.z = (gripper_off + offset);
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.x = 0.0;
        quaternion.setRPY(0.0, M_PI, M_PI/2);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);
        //Negative
        current_pose.pose.position.z = -(gripper_off + offset);
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.x = 0.0;
        quaternion.setRPY(0.0, 0.0, -M_PI/2);
        current_pose.pose.orientation.x = quaternion.x();
        current_pose.pose.orientation.y = quaternion.y();
        current_pose.pose.orientation.z = quaternion.z();
        current_pose.pose.orientation.w = quaternion.w();
        tf2::doTransform(current_pose, real_pose, transformStamped);
        pose_stamped_vector.push_back(real_pose);}
  }

  return pose_stamped_vector;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pick_and_place");
  bool homing = false;
  ros::NodeHandle node_handle("~");
  node_handle.getParam("homing", homing);

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string GRIPPER_GROUP = "gripper";


  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_interface(GRIPPER_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  const moveit::core::JointModelGroup* gripper_joints =
      gripper_interface.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  move_group_interface.setEndEffectorLink("gripper_base");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  boost::shared_ptr<geometry_msgs::Pose const> msg;

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

  // Closed Joint Trajectory
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

  // Grasp Joint Trajectory
  trajectory_msgs::JointTrajectory grasp_traj;
  grasp_traj.header.frame_id = "base_link";

  grasp_traj.joint_names.resize(2);
  grasp_traj.joint_names[0] ="joint1";
  grasp_traj.joint_names[1] ="joint2";

  grasp_traj.points.resize(1);
  grasp_traj.points[0].positions.resize(2);
  grasp_traj.points[0].time_from_start = ros::Duration(1);

  grasp_traj.points[0].positions[0] = 0.46;
  grasp_traj.points[0].positions[1] = -0.46;

  // TO OPEN GRIPPER
  // ros::Duration(1).sleep();
  // gripper.publish(open_traj);

  // TO CLOSE GRIPPER
  // ros::Duration(1).sleep();
  // gripper.publish(closed_traj);

  // BASE
  moveit_msgs::CollisionObject base;
  base = UR5Base();

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

  object_ids.push_back(base.id);
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
  table_pose.position.z = -0.112;

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
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.8;
  // box_pose.position.y = 0.3;
  // box_pose.position.z = 0.0275;

  // collision_object.primitives.push_back(box_primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  collision_objects.push_back(base);
  collision_objects.push_back(table);
  planning_scene_interface.addCollisionObjects(collision_objects);

  //Plans
  moveit::planning_interface::MoveGroupInterface::Plan pre_gpose_plan;
  moveit::planning_interface::MoveGroupInterface::Plan gpose_plan;
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  moveit::planning_interface::MoveGroupInterface::Plan post_grasp_plan;
  moveit::planning_interface::MoveGroupInterface::Plan pre_place_plan;
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;

  //TF2 Transformers
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;

  ros::Duration(1).sleep();  
  arucoTF(tf_broadcaster);

  //GUI Input
  boost::shared_ptr<sensor_msgs::Joy const> sharedGuiInput;
  sensor_msgs::Joy gui_input;

  //Object Description
  std_msgs::String target_obj;
  geometry_msgs::Pose target_pose;
  std_msgs::String target_dim;
  geometry_msgs::PoseStamped relative_pose;
  geometry_msgs::PoseStamped real_pose;

  //Grasp Pose
  std::vector<geometry_msgs::PoseStamped> grasp_pose_vector;
  std::string idx;
  geometry_msgs::PoseStamped grasp_pose;
  geometry_msgs::PoseStamped post_grasp_pose;

  //Place Pose
  geometry_msgs::PoseStamped obj_place_pose;
  geometry_msgs::PoseStamped place_pose;
  geometry_msgs::PoseStamped pre_place_pose;

  // PICK AND PLACE RUTINE
  while (ros::ok())
  {
    ROS_INFO("Waiting for message...");
    msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/pick_pose",node_handle);
    target_pose = *msg;

    move_group_interface.detachObject(collision_object.id);
    planning_scene_interface.removeCollisionObjects({collision_object.id});

    // PICK!!!
    ROS_INFO("Pick Pose Received");

    box_pose = target_pose;
    collision_object.primitives = {box_primitive};
    collision_object.primitive_poses = {box_pose};
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    
    real_pose.header.frame_id = "base_link";
    real_pose.pose = target_pose;
    geometry_msgs::TransformStamped objTS = poseStampedToTransformStamped(real_pose);
    // GET PICK POSE FROM OBJ POSE
    grasp_pose_vector = generateGraspPose(objTS, box_primitive, "obj_frame");
    // Sort the vector using the custom comparison function
    std::sort(grasp_pose_vector.begin(), grasp_pose_vector.end(), compareZPosition);

    // Visualize grasp poses
    for (int i = 0; i < grasp_pose_vector.size(); ++i)
    {
      idx = intToString(i);
      grasp_pose = grasp_pose_vector[i];
      visual_tools.publishAxisLabeled(grasp_pose.pose, "gpose_"+idx);
      visual_tools.trigger();
    }

    //Open Gripper
    gripper_interface.setJointValueTarget(open_traj.points[0].positions);
    ros::Duration(1).sleep();
    gripper_interface.move();
    gripper.publish(open_traj);

    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    moveit::core::RobotState home_state(*move_group_interface.getCurrentState());
    move_group_interface.setStartState(start_state);
    move_group_interface.setNumPlanningAttempts(10);
    //Try Grasp Poses
    for (int i = 0; i < grasp_pose_vector.size(); ++i)
    {
      // grasp_pose = grasp_pose_vector[i];

      // post_grasp_pose = grasp_pose;
      // post_grasp_pose.pose.position.z = grasp_pose.pose.position.z + 0.05;

      move_group_interface.setPoseTargets(grasp_pose_vector);
      bool success1 = (move_group_interface.plan(pre_gpose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      std::vector<double> joints_pre_grasp = pre_gpose_plan.trajectory_.joint_trajectory.points.back().positions;
      start_state.setJointGroupPositions(joint_model_group, joints_pre_grasp);
      move_group_interface.setStartState(start_state);
      move_group_interface.setPoseTarget(grasp_pose);   
      bool success2 = (move_group_interface.plan(gpose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // move_group_interface.plan(gpose_plan);
      if (success1 and success2)
      {
        visual_tools.publishTrajectoryLine(pre_gpose_plan.trajectory_, joint_model_group);
        visual_tools.publishTrajectoryLine(gpose_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

        ROS_INFO_NAMED("ur5_demo", "Press 'next' to execute planning");
        
        sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
        if(sharedGuiInput != NULL){
          gui_input = *sharedGuiInput;
        }

        if(gui_input.buttons[1]){
          move_group_interface.execute(pre_gpose_plan);
          move_group_interface.execute(gpose_plan);
          // ros::Duration(1).sleep();
          gripper_interface.setJointValueTarget(closed_traj.points[0].positions);
          //Close Gripper
          move_group_interface.attachObject(collision_object.id, "tool0", { "claw_a", "claw_b" });
          ros::Duration(0.5).sleep();
          bool success_gripper = (gripper_interface.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          // move_group_interface.plan(gpose_plan);
          if (success_gripper){
            // ros::Duration(1).sleep();
            gripper.publish(grasp_traj);
            // move_group_interface.execute(gripper_plan);
            ros::Duration(0.5).sleep();            
          }
          else {
            move_group_interface.detachObject(collision_object.id);
            ROS_INFO("Gripper Planning Failed");
          }
          //GOING HOME
          if(homing){
            bool home_not_done = true;
            while(home_not_done){
              move_group_interface.setStartStateToCurrentState();
              move_group_interface.setJointValueTarget(home_state);
              move_group_interface.plan(gpose_plan);
              visual_tools.publishTrajectoryLine(gpose_plan.trajectory_, joint_model_group);
              visual_tools.trigger();
              ROS_INFO_NAMED("ur5_demo", "Press 'next' to execute planning." );
              sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
              if(sharedGuiInput != NULL){
                gui_input = *sharedGuiInput;
              }
              if(gui_input.buttons[1]){
                move_group_interface.execute(gpose_plan);
                home_not_done = false;
              }
              else if(gui_input.buttons[2]){
                ROS_INFO_NAMED("ur5_demo", "Replanning..." );
              }
              if(gui_input.buttons[4]){
                break;
              }
            }
          }
           
          //PLACE
          ROS_INFO("Waiting for place pose...");
          msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/place_pose",node_handle);
          obj_place_pose.pose = *msg;
          obj_place_pose.header.frame_id = "base_link";

              //get the inverse transform of the object
          tf2::Transform transform;
          tf2::fromMsg(objTS.transform, transform);
          tf2::Transform inverse_transform = transform.inverse();

              // Convert back to geometry_msgs::TransformStamped
          geometry_msgs::TransformStamped inverse_transform_stamped;
          inverse_transform_stamped.header = objTS.header;
          inverse_transform_stamped.transform = tf2::toMsg(inverse_transform);

          tf2::doTransform(grasp_pose, relative_pose, inverse_transform_stamped);

          geometry_msgs::TransformStamped placeTS = poseStampedToTransformStamped(obj_place_pose);

          tf2::doTransform(relative_pose, place_pose, placeTS);

          pre_place_pose = place_pose;
          pre_place_pose.pose.position.z = place_pose.pose.position.z + 0.05; 

          visual_tools.publishAxisLabeled(place_pose.pose, "place_pose");
          visual_tools.publishAxisLabeled(pre_place_pose.pose, "pre_place_pose");
          visual_tools.publishAxisLabeled(pre_place_pose.pose, "post_grasp_pose");
          visual_tools.trigger();

          bool place_not_done = true;

          while(place_not_done){
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(post_grasp_pose);
            move_group_interface.plan(post_grasp_plan);
            visual_tools.publishTrajectoryLine(post_grasp_plan.trajectory_, joint_model_group);

            std::vector<double> joints_post_grasp = post_grasp_plan.trajectory_.joint_trajectory.points.back().positions;
            start_state.setJointGroupPositions(joint_model_group, joints_post_grasp);
            move_group_interface.setStartState(start_state);
            move_group_interface.setPoseTarget(pre_place_pose);
            move_group_interface.plan(pre_place_plan);
            visual_tools.publishTrajectoryLine(pre_place_plan.trajectory_, joint_model_group);
            
            std::vector<double> joints_pre_place = pre_place_plan.trajectory_.joint_trajectory.points.back().positions;
            start_state.setJointGroupPositions(joint_model_group, joints_pre_place);
            move_group_interface.setStartState(start_state);
            move_group_interface.setPoseTarget(place_pose);
            move_group_interface.plan(place_plan);
            visual_tools.publishTrajectoryLine(place_plan.trajectory_, joint_model_group);
            
            visual_tools.trigger();

            ROS_INFO_NAMED("ur5_demo", "Press 'next' to execute placing, press 'continue' to plan again, press 'quit' to stop trying." );
            sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
            if(sharedGuiInput != NULL){
              gui_input = *sharedGuiInput;
            }

            if(gui_input.buttons[1]){
              move_group_interface.execute(post_grasp_plan);
              move_group_interface.execute(pre_place_plan);
              move_group_interface.execute(place_plan);
              gripper_interface.setJointValueTarget(open_traj.points[0].positions);
              // ros::Duration(1).sleep();
              // gripper_interface.move();
              gripper.publish(open_traj);
              ros::Duration(0.5).sleep();
              move_group_interface.detachObject(collision_object.id);
              //GOING HOME
              bool home_not_done = true;
              while(home_not_done){
                move_group_interface.setStartStateToCurrentState();
                move_group_interface.setPoseTarget(pre_place_pose);
                move_group_interface.plan(gpose_plan);
                visual_tools.publishTrajectoryLine(gpose_plan.trajectory_, joint_model_group);
                visual_tools.trigger();
                ROS_INFO_NAMED("ur5_demo", "Press 'next' to execute planning." );
                sharedGuiInput = ros::topic::waitForMessage<sensor_msgs::Joy>("/rviz_visual_tools_gui",node_handle);
                if(sharedGuiInput != NULL){
                  gui_input = *sharedGuiInput;}
   
                if(gui_input.buttons[1]){
                  move_group_interface.execute(gpose_plan);
                  home_not_done = false;
                }
                else if(gui_input.buttons[2]){
                  ROS_INFO_NAMED("ur5_demo", "Replanning..." );
                }
                else if(gui_input.buttons[4]){
                  break;
                }
              }
            
              place_not_done = false;
            }
            else if(gui_input.buttons[2]){
              ROS_INFO_NAMED("ur5_demo", "Replanning..." );
            }
            else if(gui_input.buttons[4]){
              ROS_INFO_NAMED("ur5_demo", "Place not executed..." );
              place_not_done = false;
            }
          }
          

          break;
        }
        if(gui_input.buttons[4]){
          break;
        }
      }
      else
      {
        ROS_WARN("Motion planning failed!");
      }
    }
    

     
  }
  ros::spin();
  return 0;
}