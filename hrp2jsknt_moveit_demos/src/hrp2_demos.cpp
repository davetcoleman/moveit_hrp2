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
   Desc:   Testing program for humanoid/HRP2 functionality
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
#include <ompl_visual_tools/ompl_visual_tools.h>

// MoveIt msgs
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>

// Constraint sampler
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

// Random numbers
#include <random_numbers/random_numbers.h>

// Stability checker
#include <moveit_humanoid_stability/humanoid_stability.h>

// ROS
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/pointer_cast.hpp>

// C++
#include <limits>       // std::numeric_limits

namespace hrp2jsknt_moveit_demos
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MARKER_TOPIC = "/hrp2_visual_markers";
static const std::string BASE_LINK = "/odom";

class HRP2Demos
{
private:

  ros::NodeHandle nh_;

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

  // The visual tools for interfacing with Rviz
  ompl_visual_tools::OmplVisualToolsPtr ompl_visual_tools_;

  // Optional monitor to communicate with Rviz
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Constraint sampler
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;

  // Tool for checking balance
  moveit_humanoid_stability::HumanoidStabilityPtr humanoid_stability_;


public:
  HRP2Demos(const std::string planning_group_name)
    : nh_("~")
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
    //visual_tools_->loadTrajectoryPub();

    // Clear all old visual aspects from Rviz
    resetRviz();

    // Allow time to startup
    ros::Duration(0.1).sleep();
  }

  void resetRviz()
  {
    visual_tools_->deleteAllMarkers(); // clear all old markers
    //visual_tools_->removeAllCollisionObjects(); // clear all old collision objects that might be visible in rviz
  }

  void loadPlanningPipeline()
  {
    if (!planning_pipeline_)
    {
      // Setup planning pipeline
      planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));
    }
  }

  void loadHumanoidStabilityChecker(bool verbose)
  {
    // Configure stability checker
    if (!humanoid_stability_)
      humanoid_stability_.reset(new moveit_humanoid_stability::HumanoidStability(verbose, *robot_state_, visual_tools_));
  }

  // Whole body planning with MoveIt!
  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=1 verbose:=0 problems:=1 runs:=1 use_experience:=1 use_collisions:=0
  void genRandWholeBodyPlans(int problems, bool verbose, bool use_experience, bool use_collisions)
  {
    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return;

    // Show the lab as collision objects
    if (use_collisions)
    {
      jskLabCollisionEnvironment();
    }
    else
    {
      ros::Duration(0.25).sleep();
      visual_tools_->removeAllCollisionObjectsPS(); // clear all old collision objects that might be visible in rviz
    }

    // Always prevent feet from going into floor
    visual_tools_->publishCollisionFloor(0, "Floor");

    std::ofstream logging_file;
    logging_file.open("/home/dave/ompl_storage/lightning_logging.csv");

    // Move robot to specific place on plane
    //fixRobotStateFoot(robot_state_, 1.0, 0.5);
    //fixRobotStateFoot(goal_state_, 1.0, 0.5);

    // Create a constraint sampler for random poses
    //loadConstraintSampler(verbose);
    loadConstraintSampler(verbose);

    // Set custom validity checker for balance constraints
    loadHumanoidStabilityChecker(verbose);
    bool verbose_feasibility = true;
    planning_scene_->setStateFeasibilityPredicate(humanoid_stability_->getStateFeasibilityFn());

    // Load planning
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Loading planning pipeline");
    loadPlanningPipeline(); // always call before using generatePlan()

    // Remember the planning context even after solving is done
    planning_interface::PlanningContextPtr planning_context_handle;
    // Remember all planning context handles so we can save to file in the end
    std::set<planning_interface::PlanningContextPtr> planning_context_handles;

    ompl::tools::LightningPtr lightning;

    // Loop through planning
    for (std::size_t i = 0; i < problems; ++i)
    {
      if (!ros::ok())
        break;

      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "NEW PLAN STARTING (" << i+1 << " of " << problems << ")----------------------------------------------------- " << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;

      genRandWholeBodyPlans(verbose, use_experience, use_collisions, planning_context_handle);

      // Save all contexts to a set
      planning_context_handles.insert(planning_context_handle);
      if (planning_context_handles.size() > 1)
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Unexpected: more than 1 planning context now exists");
        exit(-1);
      }

      // Load ptrs on first pass
      if (i == 0 && (use_experience || true))
      {
        ompl_interface::ModelBasedPlanningContextPtr mbpc = boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(planning_context_handle);
        lightning = boost::dynamic_pointer_cast<ompl::tools::Lightning>(mbpc->getOMPLSimpleSetup());
      }

      // Debugging
      if (use_experience || true)
      {
        std::cout << std::endl;
        lightning->printLogs();
        lightning->saveDataLog(logging_file);
        logging_file.flush();
      }

      // Save database every 20 paths
      if ((i+1) % 20 == 0 && (use_experience || true))
      {
        ROS_WARN_STREAM_NAMED("hrp2_demos","Saving experience db...");
        lightning->saveIfChanged();
      }

    }

    logging_file.close();

    // Save the solutions to file before shutting down
    if (use_experience || true)
    {
      ROS_WARN_STREAM_NAMED("hrp2_demos","Saving experience db...");
      lightning->saveIfChanged();
    }

  }

  void genRandWholeBodyPlans(bool verbose, bool use_experience, bool use_collisions, planning_interface::PlanningContextPtr &planning_context_handle)
  {
    // Use constraint sampler to find valid random state
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Generating random start and goal states");
    int attempts = 10000;

    /*
    if (!constraint_sampler_->sample(*robot_state_, *robot_state_, attempts))
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Unable to find valid start state");
      return;
    }
    */
    static const std::string state_name = "one_foot_transition"; //one_foot_start";    
    std::cout << "Joint model group: " << joint_model_group_->getName() << std::endl;
    setStateToGroupPose(robot_state_,  state_name, joint_model_group_);

    ROS_INFO_STREAM_NAMED("temp","Starting to look for goal state...");
    while(true)
    {
      if (!constraint_sampler_->sample(*goal_state_, *goal_state_, attempts))
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Unable to find valid goal state");
        return;
      }
      // Updase virtual joint transform to fake base
      goal_state_->updateStateWithFakeBase();

      visual_tools_->publishRobotState(goal_state_);
      std::cout << "Visualizing goal state " << std::endl;
      std::cout << "Is this a good goal state? " << std::endl;
      char c = std::cin.get();
      int mode = c - '0';
      std::cout << "key: " << c << " mode: " << mode << std::endl;
      // eat enter key character
      c = std::cin.get();      

      if (mode == 1)
        break;
    }
    std::cout << "Keeping goal state " << std::endl;

    // Updase virtual joint transform to fake base
    robot_state_->updateStateWithFakeBase();

    // Visualize
    if (verbose || true)
    {
      visual_tools_->publishRobotState(robot_state_);
      std::cout << "Visualizing robot state " << std::endl;
      ros::Duration(4).sleep();

      visual_tools_->hideRobot();
    }

    // Plan to pose
    genRandWholeBodyPlan(verbose, use_experience, planning_context_handle, robot_state_, goal_state_);

    // Plan back to start
    std::cout << "Continue planning back to start?" << std::endl;
    char c = std::cin.get();
    int mode = c - '0';
    std::cout << "key: " << c << " mode: " << mode << std::endl;
    // eat enter key character
    c = std::cin.get();      

    if (mode == 1)
      genRandWholeBodyPlan(verbose, use_experience, planning_context_handle, goal_state_, robot_state_);
  }

  void genRandWholeBodyPlan(bool verbose, bool use_experience, planning_interface::PlanningContextPtr &planning_context_handle,
                            robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
  {
    // Test start state
    if( !humanoid_stability_->isValid(*start_state, true) )
      ROS_ERROR_STREAM_NAMED("temp","Not valid!!");


    // Create motion planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);

    // Goal constraint
    double tolerance_pose = 0.0001;
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(*goal_state, joint_model_group_,
                                                                                               tolerance_pose, tolerance_pose);
    req.goal_constraints.push_back(goal_constraint);

    // Other settings
    req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = planning_group_name_;
    req.num_planning_attempts = 1; // this must be one else it threads and doesn't use lightning correctly
    req.allowed_planning_time = 60*10; // second
    req.use_experience = use_experience;

    // Call pipeline
    std::vector<std::size_t> dummy;

    // SOLVE
    planning_pipeline_->generatePlan(planning_scene_, req, res, dummy, planning_context_handle);

    // Check that the planning was successful
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =======================================================");
      if (verbose)
        ROS_INFO_STREAM_NAMED("hrp2_demos","Attempting to visualize trajectory anyway...");
    }

    if (verbose || true)
    {
      // Create planning request
      moveit_msgs::MotionPlanResponse response;
      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      std::cout << "Trajectory debug:\n " << response.trajectory << std::endl;

      // Optionally publish
      if (true)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = response.trajectory.joint_trajectory;

        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > controller_action_client_;
        controller_action_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
                                        ("hrp2jsknt/follow_joint_trajectory_action", true));

        ros::spinOnce();
        ros::Duration(2).sleep();
        ros::spinOnce();
        controller_action_client_->sendGoal(goal);
      }

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //visual_tools_->hideRobot(); // hide the other robot so that we can see the trajectory TODO bug?
      visual_tools_->publishTrajectoryPath(response.trajectory, true);

    }
    else
    {
      ROS_INFO_STREAM_NAMED("hrp2_demos","Not visualizing because not in verbose mode");
    }


  }

  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=2 group:=left_arm verbose:=1
  void displayLightningPlans(int problems, bool verbose)
  {
    // All we care about is the trajectory robot
    visual_tools_->hideRobot();

    // Create a state space describing our robot's planning group
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(robot_model_, joint_model_group_);
    const ompl_interface::JointModelStateSpaceFactory factory;
    ompl_interface::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();

    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Model Based State Space has dimensions: " << model_state_space->getDimension());

    // Load lightning and its database
    ompl::tools::Lightning lightning(model_state_space);
    lightning.load(joint_model_group_->getName());

    // Get all of the paths in the database
    std::vector<ompl::base::PlannerDataPtr> paths;
    lightning.getAllPaths(paths);

    ROS_INFO_STREAM_NAMED("hrp2_demos","Number of paths to publish: " << paths.size());

    // Load the OMPL visualizer
    ompl_visual_tools_.reset(new ompl_visual_tools::OmplVisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
    ompl_visual_tools_->loadRobotStatePub("/hrp2_demos");
    ompl_visual_tools_->setStateSpace(model_state_space);

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    joint_model_group_->getEndEffectorTips(tips);
    std::cout << "Found " << tips.size() << " tips" << std::endl;

    bool show_trajectory_animated = verbose;

    // Loop through each path
    problems = !problems ? std::numeric_limits<int>::max() : problems; // if 0, show all problems
    for (std::size_t path_id = 0; path_id < std::min(int(paths.size()), problems); ++path_id)
    {
      std::cout << "Processing path " << path_id << std::endl;
      ompl_visual_tools_->publishRobotPath(paths[path_id], joint_model_group_, tips, show_trajectory_animated);
    }

  } // function

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
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Could not load left leg group");
    if (!right_leg_group)
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Could not load right leg group");

    // Start looping
    ros::Rate loop_rate(100);
    for (int counter=0; counter<40 && ros::ok(); counter++)
    {
      if (!ros::ok())
        break;

      // Hide Robot
      visual_tools_->hideRobot();
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
        ROS_INFO_STREAM_NAMED("hrp2_demos","Found IK Solution for left!");

        // Show results
        //printVirtualJointPosition(robot_state_);
        visual_tools_->publishRobotState(robot_state_);
        ros::Duration(0.5).sleep();

        // Now solve for other leg
        if (robot_state_->setFromIK(right_leg_group, right_foot_position_new, 10, 0.1))
        {
          ROS_INFO_STREAM_NAMED("hrp2_demos","Found IK Solution for BOTH!");

          // Show results
          //printVirtualJointPosition(robot_state_);
          visual_tools_->publishRobotState(robot_state_);
          ros::Duration(0.5).sleep();
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("hrp2_demos","Did not find IK solution");
          // Hide Robot
          visual_tools_->hideRobot();
          ros::Duration(0.5).sleep();
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Did not find IK solution");

        // Print out virtual joint status
        //printVirtualJointPosition(robot_state_);

        // Hide Robot
        visual_tools_->hideRobot();
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
      //ROS_WARN_STREAM_NAMED("hrp2_demos","RUN " << counter << " ------------------------------");

      // Reset
      visual_tools_->hideRobot();
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
      loadPlanningPipeline(); // always call before using generatePlan()
      planning_pipeline_->generatePlan(planning_scene_, req, res);

      // Check that the planning was successful
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("Could not compute plan successfully =======================================================");
        ROS_INFO_STREAM_NAMED("hrp2_demos","Attempting to visualize trajectory anyway...");
      }

      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("hrp2_demos","recieved trajectory: " << response.trajectory);

      visual_tools_->publishTrajectoryPath(response.trajectory, false);
    }
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

      state->updateStateWithFakeBase();
      //state->update(true); // prevent dirty transforms

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

    visual_tools_->hideRobot();

    moveit_msgs::MotionPlanResponse response;
    if (createPlanCrouch(response))
    {
      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("hrp2_demos","recieved trajectory: " << response.trajectory);
      /*
        display_trajectory_msg_.trajectory_start = response.trajectory_start;
        display_trajectory_msg_.trajectory.clear();
        display_trajectory_msg_.trajectory.push_back(response.trajectory);
        // TODO: remove and only use moveit_visual_tools
        robot_trajectory_publisher_.publish(display_trajectory_msg_);
      */
      visual_tools_->publishTrajectoryPath(response.trajectory);
      // Allow time to send trajectory
      ros::Duration(0.1).sleep();
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Failed to create plan crouch");
    }

    ros::Duration(1.0).sleep();
  }

  bool loadPlanningSceneMonitor()
  {
    // Allows us to sycronize to Rviz and also publish collision objects to ourselves
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Loading Planning Scene Monitor");
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, ROBOT_DESCRIPTION,
                                                                                   boost::shared_ptr<tf::Transformer>(), "hrp2_demos"));
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    if (planning_scene_monitor_->getPlanningScene())
    {
      // Optional monitors to start:
      bool use_octomap_monitor = false; // this prevents a /tf warning
      planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                         planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                         use_octomap_monitor);
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "test_planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Planning scene not configured");
      return false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);

    return true;
  }

  void jskLabCollisionEnvironment()
  {
    //static const std::string path = "/home/dave/2014/GSoC/planning_scenes/room73b2.scene";
    static const std::string path = "/home/dave/2014/GSoC/planning_scenes/room73b2-without-floor.scene";
    visual_tools_->loadCollisionSceneFromFile(path);
  }

  void fixRobotStateFoot(robot_state::RobotStatePtr &robot_state, double x, double y)
  {
    return; // this is now hard coded in robot_state!

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

  void testFixedFootFeature(int runs, int problems, bool verbose)
  {
    // Make random start state
    robot_state_->setToRandomPositions(joint_model_group_);
    robot_state_->updateStateWithFakeBase(); // force update
    {
      ROS_INFO_STREAM_NAMED("hrp2_demos","Publish robot ");
      visual_tools_->publishRobotState(robot_state_);
    }

    // Benchmark time
    ros::Time start_time;
    start_time = ros::Time::now();

    for (std::size_t i = 0; i < problems; ++i)
    {
      if (!ros::ok())
        return;

      // Make random start state
      robot_state_->setToRandomPositions(joint_model_group_);

      // force update
      robot_state_->updateStateWithFakeBase();

      if (verbose)
      {
        ROS_INFO_STREAM_NAMED("hrp2_demos","Publish robot ");
        visual_tools_->publishRobotState(robot_state_);
        ros::Duration(1).sleep();
      }
    }

    // Benchmark time
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds averaging " << duration/problems << " seconds per rand sample");
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
    //fixRobotStateFoot(robot_state_, 1.5, 2.0);
    //fixRobotStateFoot(robot_state_, 0.0, 0.0);

    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return;

    // Show the lab as collision objects
    //jskLabCollisionEnvironment();

    // Create a constraint sampler for random poses
    loadConstraintSampler(verbose);

    // Load a file to save the robot states to
    bool save_to_file = false;
    std::ofstream robot_states_file;
    if (save_to_file)
      robot_states_file.open("/home/dave/ros/stable_robot_states.dat");

    ros::Time start_time;
    start_time = ros::Time::now();

    for (int problem_id = 0; problem_id < problems && ros::ok(); problem_id++)
    {
      // Make random start state
      //setRandomValidState(robot_state_, joint_model_group_);

      // Display result
      if (constraint_sampler_->sample(*robot_state_, *robot_state_, attempts))
      {
        ROS_INFO_STREAM_NAMED("hrp2_demos","Found a valid sample " << problem_id);
        std::cout << std::endl;

        if (verbose || true)
        {
          ROS_INFO_STREAM_NAMED("hrp2_demos","Publish robot " << problem_id);
          visual_tools_->publishRobotState(robot_state_);
          if (verbose)
            ros::Duration(1.0).sleep();
        }

        // Save to file
        // DESIRED ORDER FOR JSK: RLEG_JOINT0  LLEG_JOINT0  CHEST_JOINT0  HEAD_JOINT0  RARM_JOINT0  LARM_JOINT0
        if (save_to_file)
        {
          std::vector< std::string > joint_groups;
          joint_groups.push_back("right_leg");
          joint_groups.push_back("left_leg");
          joint_groups.push_back("torso");
          joint_groups.push_back("head");
          joint_groups.push_back("right_arm");
          joint_groups.push_back("left_arm");
          joint_groups.push_back("left_hand");
          joint_groups.push_back("right_hand");
          robotStateToStream(*robot_state_, robot_states_file, joint_groups, problem_id == 0, " ");
        }
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("","Did not find a valid sample. Shutting down.");
        exit(0);
      }
    }

    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds. Average sample time: " << (duration/double(problems)) << " s");

    if (save_to_file)
      robot_states_file.close();
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
        ROS_WARN_STREAM_NAMED("hrp2_demos","Testing alpha with value " << alpha);

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
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Test failed");
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

    ROS_ERROR_STREAM_NAMED("hrp2_demos","Unknown planning group, no start pose found.");
    exit(-1);
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
    joint_model_group_->getEndEffectorTips(tips);

    std::vector<std::string> tip_names;
    joint_model_group_->getEndEffectorTips(tip_names);

    // Choose random end effector goal positions for left and right arm ------------------

    // Skip x number random numbers (for debugging)
    int skip_rands = 0; // set this to the id of the run you want to test
    problems += skip_rands; // offset problems by where we start from
    for (std::size_t i = 0; i < skip_rands; ++i)
    {
      ROS_WARN_STREAM_NAMED("hrp2_demos","Skipping position " << i);
      robot_state_->setToRandomPositions(joint_model_group_, *rng);
    }

    for (std::size_t problem_id = skip_rands; problem_id < problems; ++problem_id)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("hrp2_demos","Testing number " << problem_id+1 << " of " << problems << " ======================================");

      robot_state_->setToDefaultValues();
      goal_state_->setToDefaultValues();

      // Stochastically random goal positions
      robot_state_->setToRandomPositions(joint_model_group_, *rng);

      // hack to skip run 2 that is bad
      if (problem_id == 1 && false)
      {
        ROS_WARN_STREAM_NAMED("hrp2_demos","using skip run 2 hack");
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
          ROS_ERROR_STREAM_NAMED("hrp2_demos","Pose not similar: " << tips[i]->getName());
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
    loadPlanningPipeline(); // always call before using generatePlan()
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

  void printVirtualJointPosition(const robot_state::RobotStatePtr &robot_state)
  {
    ROS_INFO_STREAM_NAMED("hrp2_demos","Virtual Joint Positions:");
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

  void loadConstraintSampler(bool verbose)
  {
    // Create a constraint sampler for random poses
    moveit_msgs::Constraints constr;
    constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
    constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader());
    constraint_samplers::ConstraintSamplerManagerPtr csm = constraint_sampler_manager_loader_->getConstraintSamplerManager();
    constraint_sampler_ = csm->selectSampler(planning_scene_, planning_group_name_, constr);
    constraint_sampler_->setVerbose(verbose);

    if (!constraint_sampler_)
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","No constraint sampler loaded");
      exit(-1);
    }
    ROS_INFO_STREAM_NAMED("hrp2_demos","Chosen constraint sampler: " << constraint_sampler_->getName() );
  }

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
  bool use_experience = true;
  bool use_collisions = false;
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

    if( std::string(argv[i]).compare("--use_experience") == 0 )
    {
      ++i;
      use_experience = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using experience: " << use_experience);
    }

    if( std::string(argv[i]).compare("--use_collisions") == 0 )
    {
      ++i;
      use_collisions = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using collisions: " << use_collisions);
    }
  }

  hrp2jsknt_moveit_demos::HRP2Demos client(planning_group_name);

  while (ros::ok()) // continuously prompt user to do demos
  {
    bool loop = false;
    do
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("hrp2_demos","---------------------------------------------------------------------------------------");
      switch (mode)
      {
        case 0:
          ROS_INFO_STREAM_NAMED("hrp2_demos","0 - Quit");
          exit(0);
          break;
        case 1:
          ROS_INFO_STREAM_NAMED("hrp2_demos","1 - Whole body planning with MoveIt!");
          client.genRandWholeBodyPlans(problems, verbose, use_experience, use_collisions);
          break;
        case 2:
          ROS_INFO_STREAM_NAMED("hrp2_demos","2 - Show the experience database visually in Rviz");
          client.displayLightningPlans(problems, verbose);
          break;
        case 3:
          ROS_WARN_STREAM_NAMED("hrp2_demos","3 - Test foot fixed feature");
          client.testFixedFootFeature(runs, problems, verbose);
          break;
        case 4:
          ROS_INFO_STREAM_NAMED("hrp2_demos","4 - Plan to a pre-defined crouching position, fixed feet");
          client.genCrouching();
          break;
        case 5:
          ROS_INFO_STREAM_NAMED("hrp2_demos","5 - Solve for different fixed leg positions using KDL IK (proof of concept for sampler)");
          client.genRandLegConfigurations();
          break;
        case 6:
          ROS_INFO_STREAM_NAMED("hrp2_demos","6 - Generate random positions and plan to them with MoveIt");
          client.genRandMoveItPlan();
          break;
        case 7:
          ROS_INFO_STREAM_NAMED("hrp2_demos","7 - Sample single-foot-fixed poses of robot");
          client.genRandPoseGrounded(runs, problems, verbose);
          break;
        case 8:
          ROS_INFO_STREAM_NAMED("hrp2_demos","8 - Test single arm planning on HRP2 using MoveIt Whole Body IK solver");
          client.genIKRequestsSweep(runs, problems, seed);
          break;

        case 20:
        default:
          ROS_INFO_STREAM_NAMED("hrp2_demos","20 - Loop through all these modes continously");
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

    exit(0); // temp

    // Prompt user
    std::cout << "Last mode was " << mode << ". Next demo mode (1-x, 0 to quit):";

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
