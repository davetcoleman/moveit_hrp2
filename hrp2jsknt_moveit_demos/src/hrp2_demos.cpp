/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman
   Desc:   Testing program for recieving walking gaits for JSK lisp code
   as well as planning biped walking motions in MoveIt/OMPL
*/

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

// MoveIt msgs
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>

#include <hrp2_moveit_msgs/GetWalkingMotion.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

namespace hrp2jsknt_moveit_demos
{

static const std::string PLANNING_GROUP = "whole_body";
static const std::string ROBOT_DESCRIPTION = "robot_description";

class HRP2Demos
{
public:
  HRP2Demos(int mode)
    : nh_("~")
    , walking_service_name_("/generate_walking_service")
    , sleep_time_(0.8) // time to wait on ROS messages to clear
    , walking_client_loaded_(false)
    , robot_model_loader_(ROBOT_DESCRIPTION) // load the URDF
  {
    // Load the robot model
    robot_model_ = robot_model_loader_.getModel(); // Get a shared pointer to the robot

    // Create the planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    // Setup planning pipeline
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));

    // Get the configuration for the joints in the group
    joint_model_group_ = robot_model_->getJointModelGroup(PLANNING_GROUP);

    // Create robot states
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    robot_state_->setToDefaultValues();
    goal_state_.reset(new robot_state::RobotState(*robot_state_));

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools("/odom","/hrp2_visual_markers", robot_model_));
    visual_tools_->setLifetime(10.0);
    visual_tools_->loadRobotStatePub("/hrp2_demos");

    // Used for clearing out robot state
    createBlankState();

    // Allow time to startup
    sleep_time_.sleep();
  }

  // Use two IK solvers to find leg positions
  void genRandLegConfigurations()
  {
    /* NOTES
       z min crouch is -0.38
       z max stand is 0
    */
    robot_state_->setToDefaultValues();

    // Get state of feet
    const Eigen::Affine3d left_foot_position = robot_state_->getGlobalLinkTransform("LLEG_LINK5");
    const Eigen::Affine3d right_foot_position = robot_state_->getGlobalLinkTransform("RLEG_LINK5");
    Eigen::Affine3d left_foot_position_new;
    Eigen::Affine3d right_foot_position_new;

    // Get the joint model groups for the legs
    const robot_model::JointModelGroup* left_leg_group = robot_model_->getJointModelGroup("left_leg");
    const robot_model::JointModelGroup* right_leg_group = robot_model_->getJointModelGroup("right_leg");

    if (!left_leg_group)
      ROS_ERROR_STREAM_NAMED("demos","Could not load left leg group");
    if (!right_leg_group)
      ROS_ERROR_STREAM_NAMED("demos","Could not load right leg group");

    // Start looping
    ros::Rate loop_rate(100);
    for (int counter=0; counter<40 && ros::ok(); counter++)
    {
      if (!ros::ok())
        break;

      // Hide Robot
      hideRobot();
      ros::Duration(0.1).sleep();

      // Choose new configuration
      setStateInPlace(robot_state_);

      // Show before solving iK
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(0.5).sleep();

      // Move feet positions to be directly under torso, in a regular parallel position, squatting if necessary
      const Eigen::Affine3d &base_link = robot_state_->getJointTransform("virtual_joint");
      const Eigen::Translation3d floor_z(0,0,-base_link.translation().z());

      // First move the standard foot positions to under the current virtual_joint
      left_foot_position_new = floor_z * base_link * left_foot_position;
      // Then move the z-axis up level with the floor
      right_foot_position_new = floor_z * base_link * right_foot_position;

      // Show the desired footstep location
      visual_tools_->publishArrow(left_foot_position_new, moveit_visual_tools::RED, moveit_visual_tools::LARGE);
      visual_tools_->publishArrow(right_foot_position_new, moveit_visual_tools::GREEN, moveit_visual_tools::LARGE);

      // Now solve for one leg
      if (robot_state_->setFromIK(left_leg_group, left_foot_position_new, 10, 0.1))
      {
        ROS_INFO_STREAM_NAMED("temp","Found IK Solution for left!");

        // Show results
        //printVirtualJointPosition(robot_state_);
        visual_tools_->publishRobotState(robot_state_);
        ros::Duration(0.5).sleep();

        // Now solve for other leg
        if (robot_state_->setFromIK(right_leg_group, right_foot_position_new, 10, 0.1))
        {
          ROS_INFO_STREAM_NAMED("temp","Found IK Solution for BOTH!");

          // Show results
          //printVirtualJointPosition(robot_state_);
          visual_tools_->publishRobotState(robot_state_);
          ros::Duration(0.5).sleep();
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("temp","Did not find IK solution");
          // Hide Robot
          hideRobot();
          ros::Duration(0.5).sleep();
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("temp","Did not find IK solution");

        // Print out virtual joint status
        //printVirtualJointPosition(robot_state_);

        // Hide Robot
        hideRobot();
        ros::Duration(0.5).sleep();
      }

      loop_rate.sleep();
    }

  }

  void genRandMoveItPlan()
  {
    // Set to slightly bent knees position
    setStateToGroupPose(goal_state_,  "reset_whole_body");
    setStateToGroupPose(robot_state_, "reset_whole_body");

    // Generate random goal positions
    ros::Rate loop_rate(1);
    for (int counter=0; counter<1 && ros::ok(); counter++)
    {
      //ROS_WARN_STREAM_NAMED("temp","RUN " << counter << " ------------------------------");

      // Reset
      hideRobot();
      ros::Duration(1.0).sleep();

      // Make goal state crouching
      setStateCrouching(goal_state_);
      // Move virtual_joint in XY
      setStateComplex(goal_state_);

      // Visualize request first
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(1.0).sleep();

      visual_tools_->publishRobotState(goal_state_);
      ros::Duration(1.0).sleep();

      moveit_msgs::MotionPlanResponse response;

      // Create motion planning request
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;

      // Workspace
      req.workspace_parameters.header.frame_id = "/odom";
      // relative to center of robot
      double workspace_size = 5;
      req.workspace_parameters.min_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") - workspace_size;
      req.workspace_parameters.min_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") - workspace_size;
      req.workspace_parameters.min_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") - workspace_size;
      req.workspace_parameters.max_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") + workspace_size;
      req.workspace_parameters.max_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") + workspace_size;
      req.workspace_parameters.max_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") + workspace_size;

      // Start state
      moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

      // Path constraint - Keep the body link from rotating too much
      /*
        moveit_msgs::OrientationConstraint oc1;
        oc1.header.frame_id = "/odom";
        oc1.orientation.x = 0;
        oc1.orientation.y = 0;
        oc1.orientation.z = 0;
        oc1.orientation.w = 1;
        oc1.link_name = std::string("BODY");
        oc1.absolute_x_axis_tolerance = 0.1;
        oc1.absolute_y_axis_tolerance = 0.1;
        oc1.absolute_z_axis_tolerance = 0.1;
        oc1.weight = 1;
        req.path_constraints.orientation_constraints.push_back(oc1); // TODO: make this work
      */

      // Goal constraint
      double tolerance_pose = 0.0001;
      moveit_msgs::Constraints goal_constraint =
        kinematic_constraints::constructGoalConstraints(*goal_state_, joint_model_group_, tolerance_pose, tolerance_pose);
      req.goal_constraints.push_back(goal_constraint);

      // Other settings
      req.planner_id = "RRTConnectkConfigDefault";
      req.group_name = PLANNING_GROUP;
      req.num_planning_attempts = 1;
      req.allowed_planning_time = 30;

      // Call pipeline
      planning_pipeline_->generatePlan(planning_scene_, req, res);

      // Check that the planning was successful
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("Could not compute plan successfully =======================================================");
        ROS_INFO_STREAM_NAMED("temp","Attempting to visualize trajectory anyway...");
      }

      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("temp","recieved trajectory: " << response.trajectory);

      visual_tools_->publishTrajectoryPath(response.trajectory);

      // Allow time to send trajectory
      sleep_time_.sleep();

      ros::Duration(1.0).sleep();
    }
  }

  void genRandWalkingJumps()
  {
    // Set to crouching position
    setStateToGroupPose(goal_state_,  "reset_whole_body");
    setStateToGroupPose(robot_state_, "reset_whole_body");

    // Generate random goal positions
    ros::Rate loop_rate(1);

    for (int counter=0; counter<10 && ros::ok(); counter++)
    {
      ROS_WARN_STREAM_NAMED("temp","RUN " << counter << " ------------------------------");
      setStateXYTheta(goal_state_);
      visual_tools_->publishRobotState(robot_state_);
    }
    // let ROS send the message, then wait a while
    ros::spinOnce();
    loop_rate.sleep();

    // Copy the last goal state to our new current state
    *robot_state_ = *goal_state_;
  }

  // Send to walking server to generate footsteps
  void genRandWalking()
  {
    // Set to crouching position
    setStateToGroupPose(goal_state_,  "reset_whole_body");
    setStateToGroupPose(robot_state_, "reset_whole_body");

    // Generate random goal positions
    ros::Rate loop_rate(1);

    for (int counter=0; counter<10 && ros::ok(); counter++)
    {
      ROS_WARN_STREAM_NAMED("temp","RUN " << counter << " ------------------------------");
      setStateXYTheta(goal_state_);

      hideRobot();
      ros::Duration(2.0).sleep();
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(2.0).sleep();
      visual_tools_->publishRobotState(goal_state_);
      ros::Duration(2.0).sleep();

      hrp2_moveit_msgs::GetWalkingMotion walking_srv;
      moveit::core::robotStateToRobotStateMsg(*robot_state_, walking_srv.request.start_state);
      moveit::core::robotStateToRobotStateMsg(*goal_state_, walking_srv.request.goal_state);

      // Load the walking generator ROS service client if it is not already
      if (!walking_client_loaded_)
      {
        ros::Duration wait_for_server_time(0.0); // how long to wait for the walking server to come up

        // Create ROS Server Client
        walking_service_client_ = boost::make_shared<ros::ServiceClient>(nh_.serviceClient
                                  <hrp2_moveit_msgs::GetWalkingMotion>(walking_service_name_));
        if (!walking_service_client_->waitForExistence(wait_for_server_time))
          ROS_WARN_STREAM_NAMED("srv","Unable to connect to ROS service client with name: " << walking_service_client_->getService());
        else
          ROS_INFO_STREAM_NAMED("srv","Service client started with ROS service name: " << walking_service_client_->getService());
        walking_client_loaded_ = true;
      }

      ROS_DEBUG_STREAM("Calling service: " << walking_service_client_->getService() );
      ROS_DEBUG_STREAM_NAMED("temp","Message request: " << walking_srv.request);
      if (walking_service_client_->call(walking_srv))
      {
        trajectory_msgs::JointTrajectoryPoint last_good = walking_srv.response.trajectory.joint_trajectory.points[1];
        // Hack repair the trajectory
        for (std::size_t i = 0; i < walking_srv.response.trajectory.joint_trajectory.points.size(); ++i)
        {
          if (walking_srv.response.trajectory.joint_trajectory.points[i].positions.size() == 0)
          {
            ROS_WARN_STREAM_NAMED("temp","repairing traj point " << i);
            // repair this trajectory
            walking_srv.response.trajectory.joint_trajectory.points[i] = last_good;
          }
          else
          {
            ROS_INFO_STREAM_NAMED("temp","not repairing traj point " << i);
            // update our last provided state
            last_good = walking_srv.response.trajectory.joint_trajectory.points[i];
          }
        }

        ROS_DEBUG_STREAM("Service response recieved, message: \n" << walking_srv.response);

        if (true) // use moveit's built in trajectory publisher
        {
          // The corresponding robot state (copy from request)
          /*
          display_trajectory_msg_.trajectory_start = walking_srv.request.start_state;
          display_trajectory_msg_.trajectory.clear();
          display_trajectory_msg_.trajectory.push_back();
          // TODO: remove and only use moveit_visual_tools
          robot_trajectory_publisher_.publish(display_trajectory_msg_);
          */
          visual_tools_->publishTrajectoryPath(walking_srv.response.trajectory);
        }
        else // use our custom one
        {
          moveit_msgs::RobotTrajectory trajectory = walking_srv.response.trajectory;

          // Convert to a MoveIt datastructure
          robot_trajectory::RobotTrajectory robot_traj(robot_model_, PLANNING_GROUP);
          robot_traj.setRobotTrajectoryMsg(*robot_state_, trajectory);

          // loop through each trajectory point and display in rviz
          ROS_DEBUG_STREAM_NAMED("temp","looping though " << robot_traj.getWayPointCount() << " trajectories");

          ros::Rate loop_rate_traj(10);
          for (std::size_t traj_pt = 0; traj_pt < robot_traj.getWayPointCount(); ++traj_pt)
          {
            ROS_INFO_STREAM_NAMED("temp","trajectory point " << traj_pt << " is: \n" << robot_traj.getWayPoint(traj_pt));
            // send the message to the RobotState display
            //robot_state::robotStateToRobotStateMsg(robot_traj.getWayPoint(traj_pt), display_robot_msg_.state);
            //visual_tools_->publishRobotState( robot_traj.getWayPoint(traj_pt), PLANNING_GROUP );
            ROS_ERROR_STREAM_NAMED("temp","todo implement this (uncomment above)");

            // let ROS send the message, then wait a while
            ros::spinOnce();
            loop_rate_traj.sleep();
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("demos","No service responded, did you start the walking service?");
      }

      // let ROS send the message, then wait a while
      ros::spinOnce();
      loop_rate.sleep();

      // Copy the last goal state to our new current state
      *robot_state_ = *goal_state_;

    } // for

  }

  // Plan robot moving to crouching position
  void genCrouching()
  {
    robot_state_->setToDefaultValues();
    goal_state_->setToDefaultValues();

    // Set goal state to crouch
    setStateCrouching(goal_state_);

    // Visualize request first
    visual_tools_->publishRobotState(robot_state_);
    ros::Duration(0.5).sleep();

    visual_tools_->publishRobotState(goal_state_);
    ros::Duration(0.5).sleep();

    hideRobot();

    moveit_msgs::MotionPlanResponse response;
    if (createPlanCrouch(response))
    {
      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("temp","recieved trajectory: " << response.trajectory);
      /*
      display_trajectory_msg_.trajectory_start = response.trajectory_start;
      display_trajectory_msg_.trajectory.clear();
      display_trajectory_msg_.trajectory.push_back(response.trajectory);
      // TODO: remove and only use moveit_visual_tools
      robot_trajectory_publisher_.publish(display_trajectory_msg_);
      */
      visual_tools_->publishTrajectoryPath(response.trajectory);
      // Allow time to send trajectory
      sleep_time_.sleep();
    }

    ros::Duration(1.0).sleep();
  }

  // Show random poses
  void genRandPoses()
  {
    // loop at 1 Hz
    ros::Rate loop_rate(1);

    for (int counter=0; counter<10 && ros::ok(); counter++)
    {
      robot_state_->setToRandomPositions(joint_model_group_);

      visual_tools_->publishRobotState(robot_state_);

      // let ROS send the message, then wait a while
      loop_rate.sleep();
    }
  }

  void genRandPoseGrounded()
  {
    robot_state_->setToDefaultValues();

    // loop at 1 Hz
    ros::Rate loop_rate(1);

    // Get default z offset of foot to world coordinate
    double z_default_foot_transform = robot_state_->getGlobalLinkTransform("LLEG_LINK5").translation().z();

    for (int counter=0; counter<10 && ros::ok(); counter++)
    {
      setStateInPlace(robot_state_);

      // Reset
      hideRobot();
      ros::Duration(0.25).sleep();

      // Show original random
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(1.0).sleep();

      // Move the virtual joint such that the lowest foot touches the ground
      Eigen::Affine3d virtual_joint_transform  = robot_state_->getJointTransform("virtual_joint");

      double z_lowest_foot = std::min(
        robot_state_->getGlobalLinkTransform("LLEG_LINK5").translation().z(),
        robot_state_->getGlobalLinkTransform("RLEG_LINK5").translation().z());

      // Create the transform for moving the virtual joint
      const Eigen::Affine3d down_transform(Eigen::Translation3d(0,0,-z_lowest_foot+z_default_foot_transform));

      // Perform the transform and apply back to robot state
      virtual_joint_transform = down_transform * virtual_joint_transform;
      robot_state_->setJointPositions("virtual_joint", virtual_joint_transform);

      // Display result
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(2.0).sleep();

      // let ROS send the message, then wait a while
      loop_rate.sleep();
    }
  }

  void genSimpleArmIKRequests() // zebra
  {
    robot_state_->setToDefaultValues();
    goal_state_->setToDefaultValues();

    // Get an arm to plan with
    const robot_model::JointModelGroup* left_arm = robot_model_->getJointModelGroup("left_arm");
    robot_state_->setToRandomPositions(left_arm);

    visual_tools_->publishRobotState(robot_state_);

    // Get the end effector pose
    Eigen::Affine3d left_eef_pose  = robot_state_->getGlobalLinkTransform("LARM_LINK6");

    // Use an IK solver to find the same solution
    unsigned int attempts = 1;
    double timeout = 5;
    goal_state_->setFromIK(left_arm, left_eef_pose, attempts, timeout);

    // Show the new robot state
    ros::Duration(2).sleep();
    visual_tools_->publishRobotState(goal_state_);
  }

  void setStateXYTheta(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    x += moveit_visual_tools::VisualTools::dRand(0,0.25);
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    y += moveit_visual_tools::VisualTools::dRand(0,0.25);
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    // Rotation
    Eigen::Quaternion<float>q(
      goal_state->getVariablePosition("virtual_joint/rot_w"),
      goal_state->getVariablePosition("virtual_joint/rot_x"),
      goal_state->getVariablePosition("virtual_joint/rot_y"),
      goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::VisualTools::dRand(-20,20) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
  }

  void setStateComplex(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    //x += moveit_visual_tools::VisualTools::dRand(0,0.25);
    x = 1;
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    //y += moveit_visual_tools::VisualTools::dRand(0,0.25);
    y = 0.5;
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    /*
    // Rotation
    Eigen::Quaternion<float>q(
    goal_state->getVariablePosition("virtual_joint/rot_w"),
    goal_state->getVariablePosition("virtual_joint/rot_x"),
    goal_state->getVariablePosition("virtual_joint/rot_y"),
    goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::VisualTools::dRand(-20,20) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
    */
  }

  void setStateToGroupPose(robot_state::RobotStatePtr &state, const std::string& pose_name)
  {
    if (!state->setToDefaultValues(joint_model_group_, pose_name))
    {
      ROS_ERROR_STREAM_NAMED("demo","Failed to set pose '" << pose_name << "' for planning group '" << joint_model_group_->getName() << "'");
    }
  }

  void hideRobot()
  {
    visual_tools_->publishRobotState(blank_state_);
  }

  void setStateInPlace(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    x = moveit_visual_tools::VisualTools::dRand(-1.0,1.0);
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    y = moveit_visual_tools::VisualTools::dRand(-1.0,1.0);
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    double z = goal_state->getVariablePosition("virtual_joint/trans_z");
    z = moveit_visual_tools::VisualTools::dRand(-0.38,0.0);
    goal_state->setVariablePosition("virtual_joint/trans_z",z);

    // Rotation
    Eigen::Quaternion<float>q(
      goal_state->getVariablePosition("virtual_joint/rot_w"),
      goal_state->getVariablePosition("virtual_joint/rot_x"),
      goal_state->getVariablePosition("virtual_joint/rot_y"),
      goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::VisualTools::dRand(-15,15) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
  }

  void setStateCrouching(robot_state::RobotStatePtr &state)
  {
    std::vector<std::string> name(46);
    name[0] = "RLEG_JOINT0";
    name[1] = "RLEG_JOINT1";
    name[2] = "RLEG_JOINT2";
    name[3] = "RLEG_JOINT3";
    name[4] = "RLEG_JOINT4";
    name[5] = "RLEG_JOINT5";
    name[6] = "RLEG_JOINT6";
    name[7] = "LLEG_JOINT0";
    name[8] = "LLEG_JOINT1";
    name[9] = "LLEG_JOINT2";
    name[10] = "LLEG_JOINT3";
    name[11] = "LLEG_JOINT4";
    name[12] = "LLEG_JOINT5";
    name[13] = "LLEG_JOINT6";
    name[14] = "CHEST_JOINT0";
    name[15] = "CHEST_JOINT1";
    name[16] = "HEAD_JOINT0";
    name[17] = "HEAD_JOINT1";
    name[18] = "RARM_JOINT0";
    name[19] = "RARM_JOINT1";
    name[20] = "RARM_JOINT2";
    name[21] = "RARM_JOINT3";
    name[22] = "RARM_JOINT4";
    name[23] = "RARM_JOINT5";
    name[24] = "RARM_JOINT6";
    name[25] = "RARM_JOINT7";
    name[26] = "LARM_JOINT0";
    name[27] = "LARM_JOINT1";
    name[28] = "LARM_JOINT2";
    name[29] = "LARM_JOINT3";
    name[30] = "LARM_JOINT4";
    name[31] = "LARM_JOINT5";
    name[32] = "LARM_JOINT6";
    name[33] = "LARM_JOINT7";
    name[34] = "R_THUMBCM_Y";
    name[35] = "R_THUMBCM_P";
    name[36] = "R_INDEXMP_R";
    name[37] = "R_INDEXMP_P";
    name[38] = "R_INDEXPIP_R";
    name[39] = "R_MIDDLEPIP_R";
    name[40] = "L_THUMBCM_Y";
    name[41] = "L_THUMBCM_P";
    name[42] = "L_INDEXMP_R";
    name[43] = "L_INDEXMP_P";
    name[44] = "L_INDEXPIP_R";
    name[45] = "L_MIDDLEPIP_R";
    std::vector<double> position(46);
    position[0] = -0.00946599;
    position[1] = -0.303998;
    position[2] = -1.40749;
    position[3] = 2.30053;
    position[4] = -0.907706;
    position[5] = 0.169391;
    position[6] = 0;
    position[7] = -0.00928688;
    position[8] = -0.292467;
    position[9] = -1.3977;
    position[10] = 2.28464;
    position[11] = -0.901551;
    position[12] = 0.157853;
    position[13] = 0;
    position[14] = -0.0752511;
    position[15] = 0.952245;
    position[16] = 0;
    position[17] = 0;
    position[18] = -0.662891;
    position[19] = -0.227223;
    position[20] = 0.413513;
    position[21] = -0.41508;
    position[22] = -0.386773;
    position[23] = -0.0795726;
    position[24] = 0.0724259;
    position[25] = 0;
    position[26] = 0;
    position[27] = 0;
    position[28] = 0;
    position[29] = 0;
    position[30] = 0;
    position[31] = 0;
    position[32] = 0;
    position[33] = 0;
    position[34] = 0;
    position[35] = 0;
    position[36] = 0;
    position[37] = 0;
    position[38] = 0;
    position[39] = 0;
    position[40] = 0;
    position[41] = 0;
    position[42] = 0;
    position[43] = 0;
    position[44] = 0;
    position[45] = 0;

    // Save to goal state
    state->setVariablePositions(name,position);

    // Multi-dof joints:
    state->setVariablePosition("virtual_joint/trans_x", -0.0596537);
    state->setVariablePosition("virtual_joint/trans_y", 0.0392288);
    state->setVariablePosition("virtual_joint/trans_z", -0.363454);
    state->setVariablePosition("virtual_joint/rot_x", 0.0671162);
    state->setVariablePosition("virtual_joint/rot_y", 0.00640489);
    state->setVariablePosition("virtual_joint/rot_z", 0.00298931);
    state->setVariablePosition("virtual_joint/rot_w", 0.99772);
  }

  bool createPlanCrouch(moveit_msgs::MotionPlanResponse &response)
  {
    // Create motion planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // Workspace
    req.workspace_parameters.header.frame_id = "/odom";
    // relative to center of robot
    req.workspace_parameters.min_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") - 1;
    req.workspace_parameters.min_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") - 1;
    req.workspace_parameters.min_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") - 1;
    req.workspace_parameters.max_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") + 1;
    req.workspace_parameters.max_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") + 1;
    req.workspace_parameters.max_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") + 1;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

    // Path constraint - Keep the body link from rotating too much
    /*
      moveit_msgs::PositionConstraint pc1;
      pc1.link_name = "LLEG_LINK5";
      shape_msgs::SolidPrimitive primitive;
      primitive.type = shape_msgs::SolidPrimitive::BOX;
      primitive.dimensions.push_back(1); // x
      primitive.dimensions.push_back(1); // y
      primitive.dimensions.push_back(0.1); // z
      moveit_msgs::BoundingVolume bv1;
      bv1.primitives.push_back(primitive);
      geometry_msgs::Pose origin;
      bv1.primitive_poses.push_back(origin);
      pc1.constraint_region = bv1;
      pc1.weight = 1;
      req.path_constraints.position_constraints.push_back(pc1);
    */

    // Goal constraint
    double tolerance_pose = 0.0001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal_state_, joint_model_group_, tolerance_pose, tolerance_pose);
    req.goal_constraints.push_back(goal_constraint);

    // Other settings
    //req.planner_id = "RRTkConfigDefault"; // Regular RRT
    req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = PLANNING_GROUP;
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 30;

    // Call pipeline
    planning_pipeline_->generatePlan(planning_scene_, req, res);

    // Check that the planning was successful
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =======================================================");
      return false;
    }

    response.trajectory = moveit_msgs::RobotTrajectory();
    res.getMessage(response);
    return true;
  }

  void createBlankState()
  {
    // Copy the robot state then move it way into the distance
    blank_state_.reset(new robot_state::RobotState(*robot_state_));

    double x = blank_state_->getVariablePosition("virtual_joint/trans_x");
    x += 20;
    blank_state_->setVariablePosition("virtual_joint/trans_x",x);

    double y = blank_state_->getVariablePosition("virtual_joint/trans_y");
    y += 20;
    blank_state_->setVariablePosition("virtual_joint/trans_y",y);
  }

  void printVirtualJointPosition(const robot_state::RobotStatePtr &robot_state)
  {
    ROS_INFO_STREAM_NAMED("temp","Virtual Joint Positions:");
    const double* positions = robot_state->getJointPositions("virtual_joint");
    std::cout << "X: " << positions[0] << std::endl;
    std::cout << "Y: " << positions[1] << std::endl;
    std::cout << "Z: " << positions[2] << std::endl;
  }



private:

  ros::NodeHandle nh_;

  boost::shared_ptr<ros::ServiceClient> walking_service_client_;
  std::string walking_service_name_;

  //moveit_msgs::DisplayRobotState display_robot_msg_;
  //moveit_msgs::DisplayTrajectory display_trajectory_msg_;

  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  robot_state::RobotStatePtr goal_state_;
  robot_state::RobotStatePtr blank_state_;

  robot_model::JointModelGroup* joint_model_group_;

  planning_scene::PlanningScenePtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // For visualizing things in rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  ros::Duration sleep_time_;

  bool walking_client_loaded_; // only load when we need it
}; // class

} // namespace

int main(int argc, char **argv)
{
  // initialize random seed:
  //srand (time(NULL));

  ros::init (argc, argv, "hrp2_demos");
  ROS_INFO_STREAM_NAMED("main","Starting walking client");

  // Needed for ROS_INFO commands to work
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Parse command line arguments
  int mode = 1;
  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--mode") == 0 )
    {
      ++i;
      mode = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","In mode " << mode);
    }
  }

  hrp2jsknt_moveit_demos::HRP2Demos client(mode);

  while (ros::ok()) // continuously prompt user to do demos
  {
    bool loop = false;
    do
    {
      ROS_WARN_STREAM_NAMED("demos","-----------------------------------------------");
      switch (mode)
      {
        case 1:
          ROS_WARN_STREAM_NAMED("demos","1 - Plan to a pre-defined crouching position, fixed feet");
          client.genCrouching();
          break;
        case 2:
          ROS_WARN_STREAM_NAMED("demos","2 - Generate random walking positions and generate footsteps using ROS Service call to eulisp");
          client.genRandWalking();
          break;
        case 3:
          ROS_WARN_STREAM_NAMED("demos","3 - Generate random positions");
          client.genRandWalkingJumps();
          break;
        case 4:
          ROS_WARN_STREAM_NAMED("demos","4 - Generate random positions and plan to them with MoveIt (no walking)");
          client.genRandMoveItPlan();
          break;
        case 5:
          ROS_WARN_STREAM_NAMED("demos","5 - Solve for different fixed leg positions using KDL IK (proof of concept for sampler)");
          client.genRandLegConfigurations();
          break;
        case 6:
          ROS_WARN_STREAM_NAMED("demos","6 - Generate completely random poses of robot");
          client.genRandPoses();
        case 7:
          ROS_WARN_STREAM_NAMED("demos","7 - Generate completely random poses of robot, then transform robot to foot on ground");
          client.genRandPoseGrounded();
          break;
        case 8:
          ROS_WARN_STREAM_NAMED("demos","8 - Test single arm planning on HRP2 using KDL-variant IK solver");
          client.genSimpleArmIKRequests();
          break;
        default:
          ROS_WARN_STREAM_NAMED("demos","0 - Loop through all these modes continously");
          loop = true;
      }
      // Increment mode if desired
      if (loop)
      {
        mode++;
        if (mode > 6)
          mode = 1;
      }
    } while (loop && ros::ok());

    // Check if ROS is shutting down
    if (!ros::ok())
      break;

    // Prompt user
    std::cout << "Last mode was " << mode << ". Next demo mode (0-7, 9 to quit):";
    std::cin >> mode;
    if (mode == 9)
      break;
  }

  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();
  return 0;
}
