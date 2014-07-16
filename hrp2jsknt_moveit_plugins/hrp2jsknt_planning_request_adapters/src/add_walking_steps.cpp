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
   Desc:   Modify a plan to also have foot steps
*/

#include <ros/ros.h>

#include <class_loader/class_loader.h>

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <hrp2_moveit_msgs/GetWalkingMotion.h> // custom
#include <moveit_msgs/MoveItErrorCodes.h>

namespace hrp2jsknt_planner_request_adapters
{
static const std::string PLANNING_GROUP = "whole_body";

class AddWalkingSteps : public planning_request_adapter::PlanningRequestAdapter
{
public:

  AddWalkingSteps()
    : planning_request_adapter::PlanningRequestAdapter()
      //    , walking_client_loaded_(false)
    , walking_service_name_("/generate_walking_service")
      //, nh_("~")
  {
  }

  virtual std::string getDescription() const { return "Add Walking Steps"; }

  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index,
                            planning_interface::PlanningContextPtr &context) const
  {
    // ??
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      ROS_ERROR_STREAM_NAMED("temp","!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      ROS_DEBUG("Running '%s'", getDescription().c_str());

      moveit_msgs::MotionPlanResponse msg;
      res.getMessage(msg);

      ROS_INFO_STREAM("Current trajectory: \n" << msg);
      ROS_ERROR_STREAM("Start state: " << msg.trajectory.multi_dof_joint_trajectory.points[0].transforms[0].translation);
      ROS_ERROR_STREAM("Goal state: "  << msg.trajectory.multi_dof_joint_trajectory.points[msg.trajectory.multi_dof_joint_trajectory.points.size()-1].transforms[0].translation);

      // Convert to a MoveIt datastructure
      robot_trajectory::RobotTrajectory robot_traj(planning_scene->getRobotModel(), PLANNING_GROUP);
      robot_state::RobotState reference_state = planning_scene->getCurrentState(); // TODO is this the correct state to use?
      moveit::core::robotStateMsgToRobotState(msg.trajectory_start, reference_state);
      robot_traj.setRobotTrajectoryMsg(reference_state, msg.trajectory);

      // Temp
      bool walking_client_loaded_ = false;
      boost::shared_ptr<ros::ServiceClient> walking_service_client_;
      ros::NodeHandle nh_("~");


      // loop through each trajectory point
      ROS_DEBUG_STREAM_NAMED("temp","looping though " << robot_traj.getWayPointCount() << " trajectories");
      for (std::size_t traj_pt = 1; traj_pt < robot_traj.getWayPointCount(); ++traj_pt)
      {
        // Create the request
        hrp2_moveit_msgs::GetWalkingMotion walking_srv;

        ROS_INFO_STREAM_NAMED("temp","processing trajectory point " << traj_pt);

        // send the message to the RobotState display
        robot_state::robotStateToRobotStateMsg(robot_traj.getWayPoint(traj_pt - 1), walking_srv.request.start_state);
        robot_state::robotStateToRobotStateMsg(robot_traj.getWayPoint(traj_pt),     walking_srv.request.goal_state);

        // Load the walking generator ROS service client if it is not already
        if (!walking_client_loaded_)
        {
          ROS_DEBUG_STREAM_NAMED("temp","Loading walking service client");

          ros::Duration wait_for_server_time(1.0); // how long to wait for the walking server to come up

          // Create ROS Server Client
          walking_service_client_ = boost::make_shared<ros::ServiceClient>(nh_.serviceClient
                                    <hrp2_moveit_msgs::GetWalkingMotion>(walking_service_name_));
          if (!walking_service_client_->waitForExistence(wait_for_server_time))
          {
            ROS_WARN_STREAM_NAMED("srv","Unable to connect to ROS service client with name: " << walking_service_client_->getService());
            return false;
          }
          else
            ROS_INFO_STREAM_NAMED("srv","Service client started with ROS service name: " << walking_service_client_->getService());
          walking_client_loaded_ = true;
        }

        // Generate the walking trajectory
        ROS_DEBUG_STREAM("Calling service: " << walking_service_client_->getService() );
        ROS_INFO_STREAM_NAMED("temp","Message request: \nstart state:\n" << 
          walking_srv.request.start_state.multi_dof_joint_state.transforms[0].translation
          << " \ngoal state:\n" << walking_srv.request.goal_state.multi_dof_joint_state.transforms[0].translation );

        if ( !walking_service_client_->call(walking_srv))
        {
          ROS_ERROR_STREAM_NAMED("add_walking_steps","No service responded, did you start the walking service?");
          return false;
        }
        
        ROS_WARN_STREAM_NAMED("temp","moveit result error code is: " << walking_srv.response.error_code);
        if (walking_srv.response.error_code.val != 1) // SUCCESS
        {
          ROS_ERROR("Walking server failed");
        }

        ROS_DEBUG_STREAM("Service response recieved, message: \n Start of traj: \n" 
          << walking_srv.response.trajectory.multi_dof_joint_trajectory.points[0].transforms[0].translation
          << " \n End of traj: \n"
          << walking_srv.response.trajectory.multi_dof_joint_trajectory.points[
            walking_srv.response.trajectory.multi_dof_joint_trajectory.points.size()-1].transforms[0].translation);

        // Check that the trajectory is valid
        trajectory_msgs::JointTrajectoryPoint last_good = walking_srv.response.trajectory.joint_trajectory.points[0];
        bool is_valid = true;
        for (std::size_t i = 0; i < walking_srv.response.trajectory.joint_trajectory.points.size(); ++i)
        {
          if (walking_srv.response.trajectory.joint_trajectory.points[i].positions.size() == 0)
          {
            //Repair the trajectory if necessary - TODO remove this
            ROS_ERROR_STREAM("Missing trajectory point " << i);
            is_valid = false;
            // return false;
            // repair this trajectory
            walking_srv.response.trajectory.joint_trajectory.points[i] = last_good;
          }
          else // TODO: remove this else
          {
            //ROS_DEBUG_STREAM_NAMED("temp","not repairing traj point " << i);
            // update our last provided state
            last_good = walking_srv.response.trajectory.joint_trajectory.points[i];
          }
        }

        if (!is_valid)
        {
          //continue; // don't add this walking trajectory to our whole trajectory because it is bad
        }

        // Convert the trajectory message to a MoveIt datastructure
        robot_trajectory::RobotTrajectory walking_traj(planning_scene->getRobotModel(), PLANNING_GROUP);
        walking_traj.setRobotTrajectoryMsg(reference_state, walking_srv.response.trajectory);

        // Combine this trajetory with our main one
        for (std::size_t i = 0; i < walking_traj.getWayPointCount(); ++i)
        {
          robot_traj.insertWayPoint( traj_pt+i, walking_traj.getWayPoint(i), 0.1); // TODO set dt correctly
        }

        // Jump beyond these new trajectory points for our next point to process
        traj_pt = traj_pt + walking_traj.getWayPointCount();
        ROS_INFO_STREAM_NAMED("temp","skipping iterator to traj point " << traj_pt 
          << " out of " << robot_traj.getWayPointCount()
          << ". last walking traj was size " << walking_traj.getWayPointCount() );

      } // for traj_pts

      // Copy back to MotionPlanningResponse message
      *res.trajectory_ = robot_traj; // TODO is this memory safe?

      // Debug resulting message
      ROS_WARN_STREAM_NAMED("temp","Trajectory with walking steps added:");

      return result;
    } // if
  }

  /*
  bool walking_client_loaded_;
  boost::shared_ptr<ros::ServiceClient> walking_service_client_;
  */
  std::string walking_service_name_;

};

} // namespace

CLASS_LOADER_REGISTER_CLASS(hrp2jsknt_planner_request_adapters::AddWalkingSteps,
  planning_request_adapter::PlanningRequestAdapter);
