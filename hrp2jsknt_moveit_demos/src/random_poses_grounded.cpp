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

// Random numbers
#include <random_numbers/random_numbers.h>

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
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name);
    whole_body_group_ = robot_model_->getJointModelGroup(whole_body_group_name_);

    // Create robot states
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    robot_state_->setToDefaultValues();

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
    visual_tools_->loadRobotStatePub("/hrp2_demos");
    visual_tools_->deleteAllMarkers(); // clear all old markers

    // Allow time to startup
    sleep_time_.sleep();
  }

  bool setRandomValidState(robot_state::RobotStatePtr &state, const robot_model::JointModelGroup* jmg)
  {
    // Loop until a collision free state is found
    static const int MAX_ATTEMPTS = 100;
    int attempt_count = 0; // prevent loop for going forever
    while (attempt_count < MAX_ATTEMPTS)
    {
      // Generate random staten
      state->setToRandomPositions(jmg);

      state->update(); // prevent dirty transforms

      // Test for collision
      if (planning_scene_->isStateValid(*state, "", false))
        break;

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

  void genRandPoseGrounded(int problems)
  {
    robot_state_->setToDefaultValues();

    // loop at 1 Hz
    ros::Rate loop_rate(1);

    // Enable the robot state to have a foot base
    const robot_model::LinkModel* foot = robot_model_->getLinkModel("LLEG_LINK5");
    const Eigen::Affine3d default_foot_transform = robot_state_->getGlobalLinkTransform(foot);
    const robot_model::JointModel* start_leg_joint = robot_model_->getJointModel("RLEG_JOINT0");
    robot_state_->enableFakeBaseTransform(foot, start_leg_joint, default_foot_transform);

    for (int counter=0; counter < problems && ros::ok(); counter++)
    {
      if (!ros::ok())
        return;
      
      // Reset
      //hideRobot();
      //ros::Duration(0.25).sleep();

      // Make random start state
      if (!setRandomValidState(robot_state_, joint_model_group_))
        return;      

      // Show original random
      //visual_tools_->publishRobotState(robot_state_);
      //ros::Duration(1.0).sleep();

      // force update so that the virtual joint is updated to the grounded foot
      robot_state_->update(true);

      // Display result
      visual_tools_->publishRobotState(robot_state_);

      // let ROS send the message, then wait a while
      loop_rate.sleep();
    }
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

  hrp2jsknt_moveit_demos::HRP2Demos client("whole_body_fixed");

  ROS_INFO_STREAM_NAMED("demos","7 - Generate completely random poses of robot, then transform robot to foot on ground");
  client.genRandPoseGrounded(10);

  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();
  return 0;
}
