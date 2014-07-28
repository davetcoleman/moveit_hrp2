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
   Desc:   Custom constraint sampler for the HRP2JSKNT biped robot
   Assumes feet are fixed to ground
*/

#include <hrp2jsknt_moveit_constraint_sampler/hrp2jsknt_constraint_sampler.h>
#include <set>
#include <cassert>
#include <eigen_conversions/eigen_msg.h>
#include <boost/format.hpp>

namespace hrp2jsknt_moveit_constraint_sampler
{
bool HRP2JSKNTConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
  moveit_msgs::Constraints constraints = constr; // copy to non-const

  logError("Configure joint constraint sampler");

  // HRP2JSKNT custom constraints: define here --------------------
  moveit_msgs::JointConstraint jc1;
  moveit_msgs::OrientationConstraint oc1;

  double workspace = 1;

  // x axis orientation
  oc1.link_name = "BODY";
  oc1.header.frame_id = "BODY";
  oc1.orientation.x = 0;
  oc1.orientation.y = 0;
  oc1.orientation.z = 0;
  oc1.orientation.w = 1;
  oc1.absolute_x_axis_tolerance = 0.001;
  oc1.absolute_y_axis_tolerance = 0.001;
  oc1.absolute_z_axis_tolerance = 0.001;
  /*  TODO re-enable this
      oc1.absolute_x_axis_tolerance = 0.34906585;  // 20 degrees
      oc1.absolute_y_axis_tolerance = 0.34906585;
      oc1.absolute_z_axis_tolerance = 0.34906585;
  */
  oc1.weight = 1;
  constraints.orientation_constraints.push_back(oc1);

  // construct the joint constraints
  std::vector<kinematic_constraints::JointConstraint> jc;
  for (std::size_t i = 0 ; i < constraints.joint_constraints.size() ; ++i)
  {
    kinematic_constraints::JointConstraint j(scene_->getRobotModel());
    if (j.configure(constraints.joint_constraints[i]))
      jc.push_back(j);
  }

  // construct the *single* orientation constraint
  if (constraints.orientation_constraints.size() > 1)
  {
    logError("This constraint sampler is only able to process one orientation constraint currently");
  }
  else if (constraints.orientation_constraints.size() == 1)
  {
    // Create our one orientation constraint
    orientation_constraint_.reset(
                                  new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
    orientation_constraint_->configure(constraints.orientation_constraints[0], scene_->getTransforms());
  }

  // Load the torso (vjoint) bounds from yaml file
  ros::NodeHandle nh("~");
  static const std::string group_name = "hrp2jsknt_torso";
  nh.param(group_name + "/min_x", min_x_, 0.0);
  nh.param(group_name + "/max_x", max_x_, 0.0);
  nh.param(group_name + "/min_y", min_y_, 0.0);
  nh.param(group_name + "/max_y", max_y_, 0.0);
  nh.param(group_name + "/min_z", min_z_, 0.0);
  nh.param(group_name + "/max_z", max_z_, 0.0);
  // Add padding to all limits just in case
  const double vjoint_limit_padding = 0.1;
  min_x_ += vjoint_limit_padding;
  max_x_ += vjoint_limit_padding;
  min_y_ += vjoint_limit_padding;
  max_y_ += vjoint_limit_padding;
  min_z_ += vjoint_limit_padding;
  max_z_ += vjoint_limit_padding;

  // Get the offset between the foot and the torso to allow correct positioning of bounding box
  moveit::core::RobotState temp_state(scene_->getCurrentState());
  temp_state.setToDefaultValues();
  left_foot_to_torso_ = temp_state.getGlobalLinkTransform("LLEG_LINK5");
  std::cout << "Left foot to torso: " << std::endl;
  temp_state.printTransform(left_foot_to_torso_, std::cout);
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

  // Move the x & y bounds over from foot frame of reference to torso frame of reference. z is always w.r.t. ground (0)
  max_x_ -= left_foot_to_torso_.translation().x();
  max_y_ -= left_foot_to_torso_.translation().y();
  min_x_ -= left_foot_to_torso_.translation().x();
  min_y_ -= left_foot_to_torso_.translation().y();

  // Setup HRL Kinematics Balance Constraint
  robot_joint_group_ = scene_->getRobotModel()->getJointModelGroup("robot_joints");
    //hrl_kinematics::Kinematics::FootSupport support_mode_ = hrl_kinematics::Kinematics::SUPPORT_DOUBLE;

  for (std::size_t i = 0; i < robot_joint_group_->getVariableCount(); ++i)
  {
    // Intitialize empty
    joint_positions_map_.insert(std::make_pair(robot_joint_group_->getJointModels()[i]->getName(), 0));
  }

  return configureJoint(jc);
}

bool HRP2JSKNTConstraintSampler::configureJoint(const std::vector<kinematic_constraints::JointConstraint> &jc)
{
  clear();

  if (!jmg_)
  {
    logError("NULL planning group specified for constraint sampler");
    return false;
  }

  if (jc.empty())
  {
    sampler_name_ = "NoConstraints";
    logError("no JOINT constraints passed to joint constraint sampler 2");
    //return false;
  }
  else
    sampler_name_ = "hrp2jsk_constraint_sampler";

  logError("%s: CONFIGURE HRP2JSKNT CONSTRAINT SAMPLER", sampler_name_.c_str());

  // find and keep the constraints that operate on the group we sample
  // also keep bounds for joints for convenience
  std::map<std::string, JointInfo> bound_data;
  for (std::size_t i = 0 ; i < jc.size() ; ++i)
  {
    // Check that joint constraint is enabled
    if (!jc[i].enabled())
      continue;

    // Check that joint constraint has valid joint model
    const robot_model::JointModel *jm = jc[i].getJointModel();
    if (!jmg_->hasJointModel(jm->getName()))
      continue;

    // Get the bounds of this variable
    const robot_model::VariableBounds& joint_bounds = jm->getVariableBounds(jc[i].getJointVariableName());

    // Populate the joint info object
    JointInfo ji;
    // Check if this variable already has bounds set (for some reason_
    std::map<std::string, JointInfo>::iterator it = bound_data.find(jc[i].getJointVariableName());
    if (it != bound_data.end())
      ji = it->second;
    else
      ji.index_ = jmg_->getVariableGroupIndex(jc[i].getJointVariableName()); // copy the index of the variable with respect to the joint model group

    // Attempt to tighten the variables bounds if applicable from the constraint
    ji.potentiallyAdjustMinMaxBounds(std::max(joint_bounds.min_position_, jc[i].getDesiredJointPosition() - jc[i].getJointToleranceBelow()),
                                     std::min(joint_bounds.max_position_, jc[i].getDesiredJointPosition() + jc[i].getJointToleranceAbove()));

    // Debug
    logDebug("Bounds for %s JointConstraint are %g %g", jc[i].getJointVariableName().c_str(), ji.min_bound_, ji.max_bound_);

    // Error check
    if (ji.min_bound_ > ji.max_bound_ + std::numeric_limits<double>::epsilon())
    {
      std::stringstream cs; jc[i].print(cs);
      logError("The constraints for joint '%s' have no possible values for the joint: min_bound: %g, max_bound: %g. Failing.\n",
               jm->getName().c_str(), ji.min_bound_, ji.max_bound_);
      clear();
      return false;
    }

    // Save this new joint info
    bound_data[jc[i].getJointVariableName()] = ji;
  }

  // Copy our larger bound data structure into a more compact one
  for (std::map<std::string, JointInfo>::iterator it = bound_data.begin(); it != bound_data.end(); ++it)
    bounds_.push_back(it->second);

  // get a separate list of joints that are not bounded; we will sample these randomly
  const std::vector<const robot_model::JointModel*> &joints = jmg_->getJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
    if (bound_data.find(joints[i]->getName()) == bound_data.end() &&
        joints[i]->getVariableCount() > 0 &&
        joints[i]->getMimic() == NULL)
    {
      // check if all the vars of the joint are found in bound_data instead
      const std::vector<std::string> &vars = joints[i]->getVariableNames();
      if (vars.size() > 1)
      {
        bool all_found = true;
        for (std::size_t j = 0 ; j < vars.size() ; ++j)
          if (bound_data.find(vars[j]) == bound_data.end())
          {
            all_found = false;
            break;
          }
        if (all_found)
          continue;
      }
      unbounded_.push_back(joints[i]);
      // Get the first variable name of this joint and find its index position in the planning group
      uindex_.push_back(jmg_->getVariableGroupIndex(vars[0]));
      //logInform("Adding variable index %d for joint index %d",jmg_->getVariableGroupIndex(vars[0]), i);
    }

  values_.resize(jmg_->getVariableCount());
  is_valid_ = true;

  logWarn("%s: finished configuring joint sampler. Bounded: %d, Unbounded: %d", sampler_name_.c_str(), bounds_.size(), unbounded_.size());
  return true;
}

bool HRP2JSKNTConstraintSampler::displayBoundingBox() const
{
  if (!visual_tools_)
  {
    ROS_WARN_STREAM_NAMED("temp","visual tools not loaded");
    return false;
  }

  geometry_msgs::Point point1;
  geometry_msgs::Point point2;
  point1.x = max_x_;
  point1.y = max_y_;
  point1.z = max_z_;
  point2.x = min_x_;
  point2.y = min_y_;
  point2.z = min_z_;

  return visual_tools_->publishRectangle(point1, point2, moveit_visual_tools::TRANSLUCENT);
}

bool HRP2JSKNTConstraintSampler::sample(robot_state::RobotState &robot_state, const robot_state::RobotState & /* reference_state */,
                                        unsigned int max_attempts)
{
  if (!jmg_)
    logError("no joint model group loaded");

  if (!is_valid_)
  {
    logWarn("HRP2JSKNTConstraintSampler not configured, won't sample");
    return false;
  }

  // Visualization helper TODO: disable this
  if (!visual_tools_)
  {
    visual_tools_.reset(new moveit_visual_tools::VisualTools("/odom", "/hrp2_visual_markers", scene_->getRobotModel()));
    visual_tools_->loadRobotStatePub("/hrp2_state_sampler");
    visual_tools_->deleteAllMarkers();

    if (verbose_)
      displayBoundingBox();
  }

  logWarn("%s: HRP2JSKNTConstraintSampler SAMPLING -----------------------------",sampler_name_.c_str());

  const robot_model::JointModel *vjoint = jmg_->getJointModel("virtual_joint");
  const robot_model::LinkModel  *rfoot  = jmg_->getLinkModel("RLEG_LINK5");
  const robot_model::LinkModel  *lfoot  = jmg_->getLinkModel("LLEG_LINK5");
  //const robot_model::JointModelGroup *whole_body_fixed = robot_state.getRobotModel()->getJointModelGroup("whole_body_fixed");

  robot_state.setToDefaultValues(jmg_, "reset_whole_body");

  bool stable;
  tf::Point com;

  for (std::size_t attempt = 0; attempt < max_attempts; ++attempt)
  {
    if (verbose_)
      logInform("Sampling attempt number %d for group %s", attempt, jmg_->getName().c_str() );

    if (!ros::ok())
      return false;

    // Calculate random position of robot
    /*if (!sampleJoints(robot_state))
      {
      logError("Unable to sample joints");
      return false;
      } */

    // Generate random state
    // TODO: make this only sample the leg, then later sample the rest of the robot
    robot_state.setToRandomPositions(jmg_);

    // Generate simple movement states
    if (false)
    {
      double lleg_position[1];
      lleg_position[0] = robot_state.getJointPositions("LLEG_JOINT1")[0] + 0.01;
      robot_state.setJointPositions("LLEG_JOINT1", lleg_position);
    }

    // force update
    robot_state.updateStateWithFakeBase();

    // Check if vjoint (torso) is within reasonable limits
    const double* vjoint_positions = robot_state.getJointPositions(vjoint);
    {
      if (verbose_ && false)
      {
        std::cout << "Vjoint: " << std::endl;
        std::cout << "  X: " << boost::format("%8.4f") % min_x_ << boost::format("%8.4f") % vjoint_positions[0]
                  << boost::format("%8.4f") % max_x_ << std::endl;
        std::cout << "  Y: " << boost::format("%8.4f") % min_y_ << boost::format("%8.4f") % vjoint_positions[1]
                  << boost::format("%8.4f") % max_y_ << std::endl;
        std::cout << "  Z: " << boost::format("%8.4f") % min_z_ << boost::format("%8.4f") % vjoint_positions[2]
                  << boost::format("%8.4f") % max_z_ << std::endl;
      }
      //robot_state.getFakeBaseTransform().translation().x())
      if (vjoint_positions[0] < min_x_ ||
          vjoint_positions[0] > max_x_ ||
          vjoint_positions[1] < min_y_ ||
          vjoint_positions[1] > max_y_ ||
          vjoint_positions[2] < min_z_ ||
          vjoint_positions[2] > max_z_)
      {
        if (verbose_)
          ROS_WARN_STREAM_NAMED("temp","Sample outside VIRTUAL JOINT constraints");
        continue; // stop checking
      }
    }

    // Check if the right foot is above ground
    if (robot_state.getGlobalLinkTransform(rfoot).translation().z() < 0)
    {
      if (verbose_)
        ROS_WARN_STREAM_NAMED("temp","Sample outside RIGHT FOOT ground constraint ");
      continue; // stop checking
    }

    // Sample rest of body
    // TODO

    // Check COM balance constraints --------------------------------------------------------------
    for (std::size_t i = 0; i < robot_joint_group_->getVariableCount(); ++i)
    {
      //std::cout << "var " << i << " var name " << robot_joint_group_->getJointModels()[i]->getName()
      //          << " value " << robot_state.getJointPositions(robot_joint_group_->getJointModels()[i])[0] << std::endl;

      //joint_positions_map_.insert(std::make_pair(robot_joint_group_->getJointModels()[i]->getName(),
      //                                          robot_state.getJointPositions(robot_joint_group_->getJointModels()[i])[0]));
      joint_positions_map_[robot_joint_group_->getJointModels()[i]->getName()] = 
        robot_state.getJointPositions(robot_joint_group_->getJointModels()[i])[0];
    }

    stable = test_stability_.isPoseStable(joint_positions_map_, support_mode_, normal_vector_);
    com = test_stability_.getCOM();

    //visual_tools_->deleteAllMarkers();

    // Publish visualization to Rviz
    if (verbose_)
    {
      visualization_msgs::Marker com_marker = test_stability_.getCOMMarker();
      // Translate to world frame
      com_marker.pose =
        moveit_visual_tools::VisualTools::convertPose(moveit_visual_tools::VisualTools::convertPose(com_marker.pose) *
                                                      robot_state.getGlobalLinkTransform(com_marker.header.frame_id));

      // Change frame name to world fame
      com_marker.header.frame_id = robot_state.getRobotModel()->getModelFrame(); // odom
      com_marker.ns = "COM";
      //std::cout << "new marker: ------------- \n " << com_marker << std::endl;
      visual_tools_->publishMarker(com_marker);
    }

    // Change polygon points to world frame
    if (verbose_ && false)
    {
      const geometry_msgs::PolygonStamped polygon_msg = test_stability_.getSupportPolygon();

      std::vector<geometry_msgs::Point> points;
      for (std::size_t i = 0; i < polygon_msg.polygon.points.size(); ++i)
      {
        Eigen::Affine3d temp_pose = moveit_visual_tools::VisualTools::convertPoint32ToPose(polygon_msg.polygon.points[i]);

        //temp_pose.translation().z() = temp_pose.translation().z() - 0.1;
        robot_state.update(true);
        //robot_state.printTransform(robot_state.getGlobalLinkTransform("BODY"));
        temp_pose = temp_pose * robot_state.getGlobalLinkTransform("BODY");

        points.push_back( visual_tools_->convertPose(temp_pose).position );
      }
      points.push_back(points.front()); // connect first and last points for last line

      visual_tools_->publishPath(points);
    }

    if (verbose_ && false)
    {
      visual_tools_->publishRobotState(robot_state);
      std::cout << "PUBLISHING ROBOT STATE FROM SAMPLER " << std::endl;
      ros::Duration(1.0).sleep();
    }

    if (!stable)
    {
      if (verbose_)
        ROS_WARN("Pose is NOT stable, pCOM at %f %f", com.x(), com.y());
      continue;
    }

    if (verbose_)
      ROS_INFO("Pose is stable, pCOM at %f %f ---------------------------------------------", com.x(), com.y());

    // Check for self-collisions and collision with environment
    if (!scene_->isStateValid(robot_state, "", false))
    {
      if (verbose_)
        ROS_ERROR_STREAM_NAMED("temp","Pose not valid (self or enviornment collision)");
      continue;
    }

    // Find min/max
    if (false)
    {
      min_x_ = std::min(vjoint_positions[0], min_x_);
      max_x_ = std::max(vjoint_positions[0], max_x_);

      min_y_ = std::min(vjoint_positions[1], min_y_);
      max_y_ = std::max(vjoint_positions[1], max_y_);

      min_z_ = std::min(vjoint_positions[2], min_z_);
      max_z_ = std::max(vjoint_positions[2], max_z_);
    }

    return true;
  } // for attempts

  ROS_DEBUG_STREAM_NAMED("temp","Aborted - ran out of attempts");

  return false;
}

bool HRP2JSKNTConstraintSampler::sampleJoints(robot_state::RobotState &robot_state)
{
  // sample the unbounded joints first (in case some joint varipables are bounded)
  std::vector<double> v;
  for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
  {
    v.resize(unbounded_[i]->getVariableCount());

    if (false)
      logInform("%s: UNCONSTRAINED: Joint number %d named %s with variables %d", sampler_name_.c_str(),
                i, unbounded_[i]->getName().c_str(),v.size());

    unbounded_[i]->getVariableRandomPositions(random_number_generator_, &v[0]);

    for (std::size_t j = 0 ; j < v.size() ; ++j)
    {
      values_[uindex_[i] + j] = v[j];
    }
  }

  // enforce the constraints for the constrained components (could be all of them)
  for (std::size_t i = 0 ; i < bounds_.size() ; ++i)
  {
    if (false)
      logInform("%s: CONSTRAINED: Joint number %d named %s bounds [%f,%f]", sampler_name_.c_str(), bounds_[i].index_,
                jmg_->getVariableNames()[ bounds_[i].index_ ].c_str(),
                bounds_[i].min_bound_, bounds_[i].max_bound_);

    values_[bounds_[i].index_] = random_number_generator_.uniformReal(bounds_[i].min_bound_, bounds_[i].max_bound_);
  }

  robot_state.setJointGroupPositions(jmg_, values_);

  return true;
}

// TODO remove
bool HRP2JSKNTConstraintSampler::sampleOrientationConstraints(robot_state::RobotState &robot_state)
{
  if (!orientation_constraint_->enabled())
  {
    logWarn("Orientation constraint is not enabled, skipping");
    return false;
  }

  Eigen::Quaterniond quat;

  // sample a rotation matrix within the allowed bounds
  double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.5) * (orientation_constraint_->getXAxisTolerance()-std::numeric_limits<double>::epsilon());
  double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.5) * (orientation_constraint_->getYAxisTolerance()-std::numeric_limits<double>::epsilon());
  double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.5) * (orientation_constraint_->getZAxisTolerance()-std::numeric_limits<double>::epsilon());
  Eigen::Affine3d diff(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX())
                       * Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
  Eigen::Affine3d reqr(orientation_constraint_->getDesiredRotationMatrix() * diff.rotation());
  quat = Eigen::Quaterniond(reqr.rotation());

  // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the model
  if (orientation_constraint_->mobileReferenceFrame() && false) // TODO
  {
    logError("is mobile reference frame (?)");
    const Eigen::Affine3d &t = robot_state.getFrameTransform(orientation_constraint_->getReferenceFrame());
    Eigen::Affine3d rt(t.rotation() * quat.toRotationMatrix());
    quat = Eigen::Quaterniond(rt.rotation());
  }

  // Now set the virtual joint quaternion to this result
  robot_state.setVariablePosition("virtual_joint/rot_x", quat.x());
  robot_state.setVariablePosition("virtual_joint/rot_y", quat.y());
  robot_state.setVariablePosition("virtual_joint/rot_z", quat.x());
  robot_state.setVariablePosition("virtual_joint/rot_w", quat.w());

  return true;
}

bool HRP2JSKNTConstraintSampler::project(robot_state::RobotState &robot_state,
                                         unsigned int max_attempts)
{
  return sample(robot_state, robot_state, max_attempts);
}

void HRP2JSKNTConstraintSampler::clear()
{
  ConstraintSampler::clear();
  bounds_.clear();
  unbounded_.clear();
  uindex_.clear();
  values_.clear();
}

} //namespace
