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

// OMPL
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl_rviz_viewer/ompl_rviz_viewer.h>

// MoveIt msgs
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>

// Constraint sampler
//#include <hrp2jsknt_moveit_constraint_sampler/hrp2jsknt_constraint_sampler.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>

// Walking
#include <hrp2_moveit_msgs/GetWalkingMotion.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

// Random numbers
#include <random_numbers/random_numbers.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/pointer_cast.hpp>

namespace hrp2jsknt_moveit_demos
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MARKER_TOPIC = "/hrp2_visual_markers";
static const std::string BASE_LINK = "/odom";

class HRP2Demos
{
public:
  HRP2Demos(const std::string planning_group_name)
    : nh_("~")
    , walking_service_name_("/generate_walking_service")
    , sleep_time_(0.5) // time to wait on ROS messages to clear
    , walking_client_loaded_(false)
    , robot_model_loader_(ROBOT_DESCRIPTION) // load the URDF
    , planning_group_name_(planning_group_name)
    , whole_body_group_name_("whole_body")
  {
    // Load the robot model
    robot_model_ = robot_model_loader_.getModel(); // Get a shared pointer to the robot

    // Create the planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    // Get the configuration for the joints in the group
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    whole_body_group_ = robot_model_->getJointModelGroup(whole_body_group_name_);

    // Create robot states
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    robot_state_->setToDefaultValues();
    goal_state_.reset(new robot_state::RobotState(*robot_state_));

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
    visual_tools_->loadRobotStatePub("/hrp2_demos");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadTrajectoryPub();
    visual_tools_->deleteAllMarkers(); // clear all old markers

    // Used for clearing out robot state
    createBlankState();

    // Allow time to startup
    sleep_time_.sleep();
  }

  void loadPlanningPipeline()
  {
    if (!planning_pipeline_)
    {
      // Setup planning pipeline
      planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));
    }
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
    setStateToGroupPose(goal_state_,  "reset_whole_body_fixed", joint_model_group_);
    setStateToGroupPose(robot_state_, "reset_whole_body_fixed", joint_model_group_);

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
      req.group_name = planning_group_name_;
      req.num_planning_attempts = 1;
      req.allowed_planning_time = 30;

      // Call pipeline
      loadPlanningPipeline(); // always call first
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
    }
  }

  // Plan with MoveIt + Lightning for different arm positions
  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=3 group:=left_arm
  void genLightningPlans(int problems, bool verbose)
  {
    for (std::size_t i = 0; i < problems; ++i)
    {
      genLightningPlans(verbose);
    }
  }

  void genLightningPlans(bool verbose)
  {
    // Show the lab as collision objects
    //jskLabCollisionEnvironment();

    // Move robot to specific place on plane
    fixRobotStateFoot(robot_state_, 1.0, 0.5);
    fixRobotStateFoot(goal_state_, 1.0, 0.5);

    // Make random goal state
    if (!setRandomValidState(goal_state_, joint_model_group_))
      return;
    // Make random start state
    //if (!setRandomValidState(robot_state_, joint_model_group_))
    //  return;

    robot_state_->update(true);
    //goal_state_->update(true);

    // Visualize
    visual_tools_->publishRobotState(robot_state_);
    std::cout << "Visualizing robot state " << std::endl;
    ros::Duration(2).sleep();

    visual_tools_->publishRobotState(goal_state_);
    std::cout << "Visualizing goal state " << std::endl;
    ros::Duration(2).sleep();

    moveit_msgs::MotionPlanResponse response;

    // Create motion planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

    // Goal constraint
    double tolerance_pose = 0.0001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal_state_, joint_model_group_, tolerance_pose, tolerance_pose);
    req.goal_constraints.push_back(goal_constraint);

    // Other settings
    req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = planning_group_name_;
    req.num_planning_attempts = 1; // this must be one else it threads and doesn't use lightning correctly
    req.allowed_planning_time = 30;
    req.use_experience = false;

    // Call pipeline
    loadPlanningPipeline(); // always call first
    std::vector<std::size_t> dummy;

    // Remember the planning context even after solving is done
    planning_interface::PlanningContextPtr planning_context_handle;

    // SOLVE
    planning_pipeline_->generatePlan(planning_scene_, req, res, dummy, planning_context_handle);

    // Check that the planning was successful
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =======================================================");
      ROS_INFO_STREAM_NAMED("temp","Attempting to visualize trajectory anyway...");
    }

    if (verbose)
    {
      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("temp","recieved trajectory: " << response.trajectory);
      visual_tools_->publishTrajectoryPath(response.trajectory, true);
    }
    else
    {
      ROS_WARN_STREAM_NAMED("temp","Not visualizing because not in verbose mode");
    }

    // Save the solutions to file
    //ROS_WARN_STREAM_NAMED("temp","Saving experience db...");
    ompl_interface::ModelBasedPlanningContextPtr mbpc = boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(planning_context_handle);

    if (req.use_experience)
    {
      ompl::tools::LightningPtr lightning = boost::dynamic_pointer_cast<ompl::tools::Lightning>(mbpc->getOMPLSimpleSetup());
      lightning->saveIfChanged();
    }
  }

  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=6 group:=left_arm verbose:=1
  void displayLightningPlans(int problems, bool verbose)
  {
    robot_state_->setToDefaultValues();

    // Create a state space describing our robot's planning group
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(robot_model_, joint_model_group_);
    const ompl_interface::JointModelStateSpaceFactory factory;
    ompl_interface::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();
    //model_state_space->printSettings(std::cout);

    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Model Based State Space has dimensions: " << model_state_space->getDimension());

    // Load lightning and its database
    ompl::tools::Lightning lightning(model_state_space);
    lightning.load(joint_model_group_->getName());

    // Get all of the paths in the database
    std::vector<ompl::base::PlannerDataPtr> paths;
    lightning.getAllPaths(paths);

    ROS_INFO_STREAM_NAMED("hrp2_demos","Number of paths: " << paths.size());

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    getEndEffectorTips(tips, joint_model_group_);
    std::cout << "Found " << tips.size() << " tips" << std::endl;

    // Assume the last link in the joint model group is the tip
    const moveit::core::LinkModel *tip_link = joint_model_group_->getLinkModels().back();

    bool show_trajectory_animated = verbose;

    Eigen::Affine3d pose;
    std::vector< std::vector<geometry_msgs::Point> > paths_msgs(tips.size()); // each tip has its own path of points
    robot_trajectory::RobotTrajectoryPtr robot_trajectory;

    // Enable the robot state to have a foot base
    fixRobotStateFoot(robot_state_, 1, 1);

    for (std::size_t path_id = 0; path_id < std::min(int(paths.size()), problems); ++path_id)
    {
      std::cout << "Processing path " << path_id << std::endl;

      // Clear each tip's individual tip out
      for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
      {
        paths_msgs[tip_id].clear();
      }

      // Optionally save the trajectory
      if (show_trajectory_animated)
      {
        robot_trajectory.reset(new robot_trajectory::RobotTrajectory(robot_model_, whole_body_group_name_));
      }

      // Each state in the path
      for (std::size_t state_id = 0; state_id < paths[path_id]->numVertices(); ++state_id)
      {
        // Check if program is shutting down
        if (!ros::ok())
          return;

        // Convert to robot state
        model_state_space->copyToRobotState( *robot_state_, paths[path_id]->getVertex(state_id).getState() );
        robot_state_->update(true); // force update so that the virtual joint is updated to the grounded foot

        visual_tools_->publishRobotState(robot_state_);

        // Each tip in the robot state
        for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
        {

          // Forward kinematics
          pose = robot_state_->getGlobalLinkTransform(tips[tip_id]);

          // Optionally save the trajectory
          if (show_trajectory_animated)
          {
            robot_state::RobotState robot_state_copy = *robot_state_;
            robot_trajectory->addSuffixWayPoint(robot_state_copy, 0.05); // 1 second interval
          }

          // Debug pose
          //std::cout << "Pose: " << state_id << " of link " << tip_link->getName() << ": \n" << pose.translation() << std::endl;

          paths_msgs[tip_id].push_back( visual_tools_->convertPose(pose).position );

          // Show goal state arrow
          if (state_id == paths[path_id]->numVertices() -1)
          {
            visual_tools_->publishArrow( pose, moveit_visual_tools::BLACK );
          }
        }
      }

      for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
      {
        visual_tools_->publishPath( paths_msgs[tip_id], moveit_visual_tools::RAND, moveit_visual_tools::SMALL );
        ros::Duration(0.05).sleep();
        visual_tools_->publishSpheres( paths_msgs[tip_id], moveit_visual_tools::BLUE, moveit_visual_tools::SMALL );
        ros::Duration(0.05).sleep();
      }

      // Debugging - Convert to trajectory
      if (show_trajectory_animated)
      {
        visual_tools_->publishTrajectoryPath(*robot_trajectory, true);
      }

    } // for each path

  }

  bool setRandomValidState(robot_state::RobotStatePtr &state, const robot_model::JointModelGroup* jmg)
  {
    // Loop until a collision free state is found
    static const int MAX_ATTEMPTS = 500;
    static int total_calls = 0;
    static int total_attempts = 0;
    total_calls++;

    int attempt_count = 0; // prevent loop for going forever
    while (attempt_count < MAX_ATTEMPTS)
    {
      // Generate random stat
      state->setToRandomPositions(jmg);

      state->update(true); // prevent dirty transforms

      // Test for collision
      if (planning_scene_->isStateValid(*state, "", false))
      {
        total_attempts += attempt_count;
        std::cout << "This sample took " << attempt_count << " with overall average " << (total_attempts / total_calls) << std::endl;
        break;
      }

      attempt_count ++;
    }

    // Explain if no valid rand state found
    if (attempt_count >= MAX_ATTEMPTS)
    {
      ROS_WARN("Unable to find a random collision free configuration after %d attempts", MAX_ATTEMPTS);
      return false;
    }

    return true;
  }

  // Send to walking server to generate footsteps
  void genRandWalking()
  {
    // Set to crouching position
    setStateToGroupPose(goal_state_,  "reset_whole_body", joint_model_group_);
    setStateToGroupPose(robot_state_, "reset_whole_body", joint_model_group_);

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
          visual_tools_->publishTrajectoryPath(walking_srv.response.trajectory);
        }
        else // use our custom one
        {
          moveit_msgs::RobotTrajectory trajectory = walking_srv.response.trajectory;

          // Convert to a MoveIt datastructure
          robot_trajectory::RobotTrajectory robot_traj(robot_model_, planning_group_name_);
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
    ros::Duration(2).sleep();

    visual_tools_->publishRobotState(goal_state_);
    ros::Duration(2).sleep();

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
    else
    {
      ROS_ERROR_STREAM_NAMED("temp","Failed to create plan crouch");
    }

    ros::Duration(1.0).sleep();
  }

  void genRandPoseGroundedZ() // grounds just along the z axis
  {
    robot_state_->setToDefaultValues();

    // loop at 1 Hz
    ros::Rate loop_rate(1);

    // Get default z offset of foot to world coordinate
    double z_default_foot_transform = robot_state_->getGlobalLinkTransform("LLEG_LINK5").translation().z();

    for (int counter=0; counter<10 && ros::ok(); counter++)
    {
      // Move the virtual joint slightly
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

  bool loadPlanningSceneMonitor()
  {
    // Allows us to sycronize to Rviz and also publish collision objects to ourselves
    std::cout << std::endl;
    std::cout << "------------------------------------------------------------------------- " << std::endl;
    std::cout << "Loading Planning Scene Monitor " << std::endl;
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, ROBOT_DESCRIPTION,
                                                                                   boost::shared_ptr<tf::Transformer>(), "hrp2_demos"));
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    if (planning_scene_monitor_->getPlanningScene())
    {
      // Optional monitors to start:
      planning_scene_monitor_->startWorldGeometryMonitor();
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "test_planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("temp","Planning scene not configured");
      return false;
    }
    std::cout << "------------------------------------------------------------------------- " << std::endl;
    std::cout << std::endl;

    return true;
  }

  void randomCollisionEnvironment()
  {
    // Move a wall
    ROS_WARN_STREAM_NAMED("temp","publishing collision wall");


    moveit_msgs::CollisionObject collision_obj;
    visual_tools_->getCollisionWallMsg(visual_tools_->fRand(0.1, 1), // x
                                       visual_tools_->fRand(0.1, 1), // y
                                       0, // angle
                                       2, // width
                                       "wall1", // name
                                       collision_obj);

    planning_scene_->processCollisionObjectMsg(collision_obj);
  }

  void jskLabCollisionEnvironment()
  {
    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return;

    // Collision
    ros::Duration(0.1).sleep();
    {
      planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
      if (ps)
      {
        //static const std::string path = "/home/dave/2014/GSoC/planning_scenes/room73b2.scene";
        static const std::string path = "/home/dave/2014/GSoC/planning_scenes/room73b2-without-floor.scene";

        std::ifstream fin(path.c_str());
        if (fin.good())
        {
          ps->loadGeometryFromStream(fin);
          fin.close();
          ROS_INFO("Loaded scene geometry from '%s'", path.c_str());
        }
        else
          ROS_WARN("Unable to load scene geometry from '%s'", path.c_str());
      }
      else
        ROS_WARN_STREAM_NAMED("temp","Unable to get locked planning scene RW");
    }
    planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  }

  void fixRobotStateFoot(robot_state::RobotStatePtr &robot_state, double x, double y)
  {
    robot_state->setToDefaultValues();

    // Enable the robot state to have a foot base
    const robot_model::LinkModel* foot = robot_model_->getLinkModel("LLEG_LINK5");
    const robot_model::JointModel* start_leg_joint = robot_model_->getJointModel("LLEG_JOINT0");
    Eigen::Affine3d default_foot_transform = robot_state->getGlobalLinkTransform(foot); // get the orientation
    //robot_state->printTransform(default_foot_transform, std::cout);

    // Change the translation transform component
    default_foot_transform.translation().x() = x;
    default_foot_transform.translation().y() = y;
    default_foot_transform.translation().z() = 0.1; //0.15 for floor obstacle
    //default_foot_transform.translation().z() = 0.0; //0.15 for floor obstacle
 
    // Set robot_state to maintain this location
    robot_state->enableFakeBaseTransform(foot, start_leg_joint, default_foot_transform);
  }

  /**
   * \brief Sample a full body's joint positions randomly for sampling based planning
   * \param runs - how many attempts to do each sample
   * \param problems - how many times to sample
   * \param verbose
   */
  void genRandPoseGrounded(int runs, int problems, bool verbose)
  {
    int attempts = runs;

    robot_state_->setToDefaultValues();

    // Move robot to specific place on plane
    //fixRobotStateFoot(robot_state_, 1.0, 0.5);
    fixRobotStateFoot(robot_state_, 0.0, 0.0);

    // Show the lab as collision objects
    //jskLabCollisionEnvironment();

    // Create a constraint sampler for random poses
    moveit_msgs::Constraints constr;
    constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
    constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader());
    constraint_samplers::ConstraintSamplerManagerPtr csm = constraint_sampler_manager_loader_->getConstraintSamplerManager();
    constraint_samplers::ConstraintSamplerPtr cs = csm->selectSampler(planning_scene_, planning_group_name_, constr);
    cs->setVerbose(verbose);

    if (!cs)
    {
      ROS_ERROR_STREAM_NAMED("temp","No constraint sampler loaded");
      exit(-1);
    }
    ROS_INFO_STREAM_NAMED("temp","Chosen constraint sampler: " << cs->getName() );

    ros::Time start_time;
    start_time = ros::Time::now();

    for (int problem_id = 0; problem_id < problems && ros::ok(); problem_id++)
    {
      // Reset
      //hideRobot();
      //ros::Duration(0.25).sleep();

      // Make random start state
      //setRandomValidState(robot_state_, joint_model_group_);      
     
      // Display result
      if (cs->sample(*robot_state_, *robot_state_, attempts))
      {
        ROS_INFO_STREAM_NAMED("temp","Found a valid sample " << problem_id);
        std::cout << std::endl;
        
        if (verbose)
        {
          ROS_INFO_STREAM_NAMED("hrp2_demos","Publish robot " << problem_id);
          visual_tools_->publishRobotState(robot_state_);
          ros::Duration(2.0).sleep();
        }
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("","Did not find a valid sample");
        exit(-1);
      }
    }

    cs->getName();

    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds. Average sample time: " << (duration/double(problems)) << " s");
  }

  // Set every joint in the group to the same joint value
  void setGroupToValue(robot_state::RobotStatePtr robot_state, const robot_model::JointModelGroup* jmg, double value)
  {
    double group_joints[jmg->getVariableCount()]; // TODO get joint, not variable
    for (std::size_t i = 0; i < jmg->getVariableCount(); ++i)
      group_joints[i] = value;
    robot_state->setJointGroupPositions(jmg, group_joints);
    robot_state->enforceBounds();
  }

  void genIKRequestsSweep(int runs, int problems, std::size_t seed)
  {
    // Whether to do parameter sweeping
    if (false)
    {
      // Save to file
      std::ofstream istream("/home/dave/ros/ik_data/times.dat");

      double test_values[7] = { 0.0, 0.0001, 0.001, 0.05, 0.01, 0.05, 0.1};

      // Vary alphas
      //for (double alpha = 0.0; alpha < 1.0; alpha += 0.05)
      for (std::size_t i = 0; i < 7; ++i)
      {
        double alpha = test_values[i];
        ROS_WARN_STREAM_NAMED("temp","Testing alpha with value " << alpha);

        // Set the alpha on the ROS param server

        //double alpha = 0.6;
        nh_.setParam("alpha", alpha);
        ros::Duration(0.1).sleep();

        double duration = genIKRequestsBenchmark(runs, problems, seed);

        // Save
        istream << alpha << "," << duration << std::endl;
      }

      istream.close();
    }
    else
    {
      genIKRequestsBenchmark(runs, problems, seed);
    }
  }

  double genIKRequestsBenchmark(int runs, int problems, std::size_t seed)
  {
    // Benchmark time
    ros::Time start_time;
    start_time = ros::Time::now();

    for (std::size_t i = 0; i < runs; ++i)
    {
      // Run test
      if (!genIKRequests(problems, seed))
      {
        ROS_ERROR_STREAM_NAMED("temp","Test failed");
        return 1000; // a large number
      }
    }

    // Benchmark time
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds");

    return duration;
  }

  // Get the starting pose that corresponds to selected planning group
  std::string getStartPose(const std::string& planning_group)
  {
    if (planning_group == "left_arm")
    {
      return "left_arm_ik_default";
    }
    else if (planning_group == "whole_body_fixed")
    {
      return "reset_whole_body_fixed";
    }
    else if (planning_group == "whole_body")
    {
      return "reset_whole_body";
    }
    else if (planning_group == "upper_body")
    {
      return "upper_body_ik_default";
    }

    ROS_ERROR_STREAM_NAMED("temp","Unknown planning group, no start pose found.");
    exit(-1);
  }

  bool getEndEffectorTips(std::vector<std::string> &tips, const robot_model::JointModelGroup* joint_model_group)
  {
    // Get a vector of tip links
    std::vector<const robot_model::LinkModel*> tip_links;
    if (!getEndEffectorTips(tip_links, joint_model_group))
      return false;

    // Convert to string names
    tips.clear();
    for (std::size_t i = 0; i < tip_links.size(); ++i)
    {
      tips.push_back(tip_links[i]->getName());
    }
    return true;
  }

  bool getEndEffectorTips(std::vector<const robot_model::LinkModel*> &tips, const robot_model::JointModelGroup* joint_model_group)
  {
    for (std::size_t i = 0; i < joint_model_group->getAttachedEndEffectorNames().size(); ++i)
    {
      const robot_model::JointModelGroup *eef = robot_model_->getEndEffector(joint_model_group->getAttachedEndEffectorNames()[i]);
      if (!eef)
      {
        ROS_ERROR_STREAM_NAMED("temp","Unable to find joint model group for eef");
        return false;
      }
      const std::string &eef_parent = eef->getEndEffectorParentGroup().second;

      const robot_model::LinkModel* eef_link = robot_model_->getLinkModel(eef_parent);
      if (!eef_link)
      {
        ROS_ERROR_STREAM_NAMED("temp","Unable to find end effector link for eef");
        return false;
      }

      tips.push_back(eef_link);
    }
    return true;
  }

  bool genIKRequests(int problems, std::size_t seed)
  {
    // Get the starting pose that corresponds to selected planning group
    std::string start_pose = getStartPose(planning_group_name_);

    // Random number generator -----------------------------------------------------
    random_numbers::RandomNumberGenerator *rng;

    // Create random number generator that is stochastic
    if (seed == 0) // use random seed
    {
      rng = new random_numbers::RandomNumberGenerator();
      ROS_INFO_STREAM_NAMED("genSimpleArmIKRequets","Used random seed: " << rng->getFirstSeed() );
    }
    else
    {
      rng = new random_numbers::RandomNumberGenerator(seed);
    }

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    getEndEffectorTips(tips, joint_model_group_);

    std::vector<std::string> tip_names;
    getEndEffectorTips(tip_names, joint_model_group_);

    // Choose random end effector goal positions for left and right arm ------------------

    // Skip x number random numbers (for debugging)
    int skip_rands = 0; // set this to the id of the run you want to test
    problems += skip_rands; // offset problems by where we start from
    for (std::size_t i = 0; i < skip_rands; ++i)
    {
      ROS_WARN_STREAM_NAMED("temp","Skipping position " << i);
      robot_state_->setToRandomPositions(joint_model_group_, *rng);
    }

    for (std::size_t problem_id = skip_rands; problem_id < problems; ++problem_id)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("temp","Testing number " << problem_id+1 << " of " << problems << " ======================================");

      robot_state_->setToDefaultValues();
      goal_state_->setToDefaultValues();

      // Stochastically random goal positions
      robot_state_->setToRandomPositions(joint_model_group_, *rng);

      // hack to skip run 2 that is bad
      if (problem_id == 1 && false)
      {
        ROS_WARN_STREAM_NAMED("temp","using skip run 2 hack");
        continue;
      }

      // Set the seed value from SRDF
      setStateToGroupPose(goal_state_, start_pose, joint_model_group_);

      // Check that the new state is valid
      robot_state_->enforceBounds();
      if (!robot_state_->satisfiesBounds(joint_model_group_))
      {
        ROS_ERROR_STREAM_NAMED("setGroupToValue","New joint values do not satisfy bounds for group " << joint_model_group_->getName());
        exit(-1);
      }

      // Debug seed joint values:
      if (true)
      {
        std::vector<double> joints(joint_model_group_->getVariableCount());
        robot_state_->copyJointGroupPositions(joint_model_group_, joints);

        if (true)
        {
          // Show desired joint values
          for (std::size_t i = 0; i < joints.size(); ++i)
          {
            std::cout << joints[i] << ", ";
          }
          std::cout << std::endl;
        }

        double epsilon = 0.05;
        for (std::size_t j = 0; j < joints.size(); ++j)
        {
          if (joints[j] < joint_model_group_->getJointModels()[j]->getVariableBounds()[0].min_position_ + epsilon)
            std::cout << " LOW " << joint_model_group_->getJointModels()[j]->getVariableBounds()[0].min_position_ << std::endl;

          if (joints[j] > joint_model_group_->getJointModels()[j]->getVariableBounds()[0].max_position_ - epsilon)
            std::cout << " HIGH " << joint_model_group_->getJointModels()[j]->getVariableBounds()[0].max_position_ << std::endl;
        }
      }

      // Visualize
      visual_tools_->publishRobotState(robot_state_);

      // Get the end effector poses
      EigenSTL::vector_Affine3d poses;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        poses.push_back( robot_state_->getGlobalLinkTransform(tips[i]) );
      }

      // Use an IK solver to find the same solution
      unsigned int attempts = 1;
      double timeout = 0; // unset

      // IK Solver
      if (!goal_state_->setFromIK(joint_model_group_, poses, tip_names, attempts, timeout))
      {
        ROS_ERROR_STREAM_NAMED("demos","setFromIK failed");
        return false;
      }

      // Error check that the values are the same
      EigenSTL::vector_Affine3d poses_new;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        poses_new.push_back( goal_state_->getGlobalLinkTransform(tips[i]) );
      }

      bool passed = true;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        if (!poseIsSimilar(poses[i], poses_new[i]))
        {
          ROS_ERROR_STREAM_NAMED("temp","Pose not similar: " << tips[i]->getName());
          passed = false;
        }
      }

      if (!passed)
      {
        std::cout << "=========================================================== " << std::endl;
        ROS_ERROR_STREAM_NAMED("temp","POSES ARE NOT SIMILAR, BENCHMARK FAILED on test " << problem_id);
        std::cout << "=========================================================== " << std::endl;
        return false;
      }
      else
      {
        // Show the new robot state
        //ros::Duration(0.25).sleep();
        //visual_tools_->publishRobotState(goal_state_);
      }

    } // for problems

    return true;
  }

  bool poseIsSimilar(const Eigen::Affine3d &pose1, const Eigen::Affine3d &pose2)
  {
    Eigen::Quaterniond quat1 = Eigen::Quaterniond(pose1.rotation());
    Eigen::Quaterniond quat2 = Eigen::Quaterniond(pose2.rotation());

    geometry_msgs::Pose p1 = moveit_visual_tools::VisualTools::convertPose(pose1);
    geometry_msgs::Pose p2 = moveit_visual_tools::VisualTools::convertPose(pose2);

    //double similarity_threshold = 0.01;
    if (
        abs(p1.position.x - p2.position.x) > std::numeric_limits<double>::epsilon() ||
        abs(p1.position.y - p2.position.y) > std::numeric_limits<double>::epsilon() ||
        abs(p1.position.z - p2.position.z) > std::numeric_limits<double>::epsilon() ||
        quat1.angularDistance(quat2) > 3 //std::numeric_limits<double>::epsilon()
        //      quat1.dot(quat2) < 1 - 100*std::numeric_limits<double>::epsilon() //using "dot" avoids a trigonometric function.
        )
    {
      if (abs(p1.position.x - p2.position.x) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff x: " << std::setprecision(12) << abs(p1.position.x - p2.position.x)  << std::endl;
      if (abs(p1.position.y - p2.position.y) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff y: " << std::setprecision(12) << abs(p1.position.y - p2.position.y)  << std::endl;
      if (abs(p1.position.z - p2.position.z) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff z: " << std::setprecision(12) << abs(p1.position.z - p2.position.z)  << std::endl;

      if (quat1.dot(quat2) < 1 - std::numeric_limits<double>::epsilon()) //using "dot" avoids a trigonometric function.
      {
        std::cout << "Difference in angle is greater than epsilon: " << std::setprecision(12) << quat1.dot(quat2) << std::endl;
      }
      if (quat1.angularDistance(quat2) > 3 )
      {
        std::cout << "Angular Distance is too large: " << std::setprecision(12) << quat1.angularDistance(quat2) << std::endl;
      }

      std::cout << "pose1 \n" << p1 <<std::endl;
      std::cout << "pose2 \n" << p2 <<std::endl;

      return false;
    }
    return true;
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

  void setStateToGroupPose(robot_state::RobotStatePtr &state, const std::string& pose_name, const robot_model::JointModelGroup* jmg)
  {
    if (!state->setToDefaultValues(jmg, pose_name))
    {
      ROS_ERROR_STREAM_NAMED("demo","Failed to set pose '" << pose_name << "' for planning group '" << jmg->getName() << "'");
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
    req.group_name = planning_group_name_;
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 30;

    // Call pipeline
    loadPlanningPipeline(); // always call first
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
    std::cout << "Position: " << std::endl;
    std::cout << "X: " << positions[0] << std::endl;
    std::cout << "Y: " << positions[1] << std::endl;
    std::cout << "Z: " << positions[2] << std::endl;
    std::cout << "Quaternion: " << std::endl;
    std::cout << "X: " << positions[3] << std::endl;
    std::cout << "Y: " << positions[4] << std::endl;
    std::cout << "Z: " << positions[5] << std::endl;
    std::cout << "W: " << positions[6] << std::endl;
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

  robot_model::JointModelGroup* joint_model_group_; // selected by user
  std::string planning_group_name_; // allow to change planning group from command line

  robot_model::JointModelGroup* whole_body_group_; // hard-coded
  std::string whole_body_group_name_; // hard-coded group for the whole robot including virtual joint

  planning_scene::PlanningScenePtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // For visualizing things in rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  ros::Duration sleep_time_;

  bool walking_client_loaded_; // only load when we need it

  // The visual tools for interfacing with Rviz
  //ompl_rviz_viewer::OmplRvizViewerPtr ompl_viewer_;

  // Optional monitor to communicate with Rviz
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

}; // class

} // namespace

int main(int argc, char **argv)
{
  // initialize random seed:
  //srand (time(NULL));

  ros::init (argc, argv, "hrp2_demos");
  ROS_INFO_STREAM_NAMED("main","Starting HRP2 Demos");

  // Needed for ROS_INFO commands to work
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Parse command line arguments
  int mode = 1;
  int runs = 1; // how many times to run the same problem
  int problems = 1; // how many problems to solve
  bool verbose = false;
  std::string planning_group_name = "whole_body";
  std::size_t seed = 0;

  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--mode") == 0 )
    {
      ++i;
      mode = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","In mode " << mode);
    }

    if( std::string(argv[i]).compare("--verbose") == 0 )
    {
      ++i;
      verbose = atoi(argv[i]); // converts to int
      if (verbose)
        ROS_INFO_STREAM_NAMED("main","Verbose is true");
    }

    if( std::string(argv[i]).compare("--runs") == 0 )
    {
      ++i;
      runs = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Performing " << runs << " runs");
    }

    if( std::string(argv[i]).compare("--problems") == 0 )
    {
      ++i;
      problems = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Performing " << problems << " problems");
    }

    if( std::string(argv[i]).compare("--seed") == 0 )
    {
      ++i;
      seed = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using seed " << seed);
    }

    if( std::string(argv[i]).compare("--group") == 0 )
    {
      ++i;
      planning_group_name = argv[i];
      ROS_INFO_STREAM_NAMED("main","Using planning group " << planning_group_name);
    }
  }

  hrp2jsknt_moveit_demos::HRP2Demos client(planning_group_name);

  while (ros::ok()) // continuously prompt user to do demos
  {
    bool loop = false;
    do
    {
      ROS_INFO_STREAM_NAMED("demos","-----------------------------------------------");
      switch (mode)
      {
        case 1:
          ROS_INFO_STREAM_NAMED("demos","1 - Plan to a pre-defined crouching position, fixed feet");
          client.genCrouching();
          break;
        case 2:
          ROS_INFO_STREAM_NAMED("demos","2 - Generate random walking positions and generate footsteps using ROS Service call to eulisp");
          client.genRandWalking();
          break;
        case 3:
          ROS_INFO_STREAM_NAMED("demos","3 - Plan with MoveIt + Lightning for different arm positions");
          client.genLightningPlans(problems, verbose);
          break;
        case 4:
          ROS_INFO_STREAM_NAMED("demos","6 - Show the experience database visually in Rviz");
          client.displayLightningPlans(problems, verbose);
          break;
        case 5:
          ROS_INFO_STREAM_NAMED("demos","5 - Solve for different fixed leg positions using KDL IK (proof of concept for sampler)");
          client.genRandLegConfigurations();
          break;
        case 6:
          ROS_INFO_STREAM_NAMED("demos","4 - Generate random positions and plan to them with MoveIt (no walking)");
          client.genRandMoveItPlan();
          break;
        case 7:
          ROS_INFO_STREAM_NAMED("demos","7 - Generate completely random poses of robot, then transform robot to foot on ground");
          client.genRandPoseGrounded(runs, problems, verbose);
          break;
        case 8:
          ROS_INFO_STREAM_NAMED("demos","8 - Test single arm planning on HRP2 using MoveIt Whole Body IK solver");
          client.genIKRequestsSweep(runs, problems, seed);
          break;
        case 0:
        default:
          ROS_INFO_STREAM_NAMED("demos","0 - Loop through all these modes continously");
          loop = true;
      }

      // Increment mode if desired
      if (loop)
      {
        mode++;
        if (mode > 8)
          mode = 1;
      }
    } while (loop && ros::ok());


    // Check if ROS is shutting down
    if (!ros::ok())
      break;
    std::cout << "debug final" << std::endl;
    return 0;
    exit(0); // temp

    // Prompt user
    std::cout << "Last mode was " << mode << ". Next demo mode (0-8, 9 to quit):";

    bool valid_mode = false;
    while (!valid_mode)
    {
      // Account for just an enter key being pressed:
      char c = std::cin.get();
      if (c == '\n')
      {
        std::cout << "ENTER KEY PRESSED " << std::endl;
      }
      else
      {
        mode = c - '0';
        std::cout << "key: " << c << " mode: " << mode << std::endl;
        // eat enter key character
        c = std::cin.get();
      }

      // make sure mode is valid
      if (mode >= 0 && mode <= 9)
        break;
      else
        std::cout << "Invalid mode: " << mode << std::endl;
    }

    // Exit program
    if (mode == 9)
      break;
  }

  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();
  return 0;
}
