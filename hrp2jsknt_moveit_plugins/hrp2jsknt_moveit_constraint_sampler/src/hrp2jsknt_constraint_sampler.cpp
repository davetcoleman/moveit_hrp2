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
#include <boost/bind.hpp>

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

  // X
  jc1.joint_name = "virtual_joint/trans_x";
  jc1.position = 0.0; // the default location
  jc1.tolerance_above = workspace;
  jc1.tolerance_below = workspace;
  jc1.weight = 1;
  constraints.joint_constraints.push_back(jc1);

  // Y
  jc1.joint_name = "virtual_joint/trans_y";
  jc1.position = 0.0; // the default location
  jc1.tolerance_above = workspace;
  jc1.tolerance_below = workspace;
  jc1.weight = 1;
  constraints.joint_constraints.push_back(jc1);

  // Z
  jc1.joint_name = "virtual_joint/trans_z";
  jc1.position = 0.0; // the default location
  jc1.tolerance_above = 0; // hrp2 cannot jump
  jc1.tolerance_below = 0.38; // min he can crouch
  jc1.weight = 1;
  constraints.joint_constraints.push_back(jc1);

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

  return configure(jc);
}

bool HRP2JSKNTConstraintSampler::configure(const std::vector<kinematic_constraints::JointConstraint> &jc)
{
  clear();

  if (!jmg_)
  {
    logError("NULL group specified for constraint sampler");
    return false;
  }

  if (jc.empty())
  {
    sampler_name_ = "NOC";
    //logError("no constraints passed to joint constraint sampler 2");
    //return false;
  }
  else
    sampler_name_ = "CON";

  logError("%s CONFIGURE HRP2JSKNT CONSTRAINT SAMPLER", sampler_name_.c_str());

  // find and keep the constraints that operate on the group we sample
  // also keep bounds for joints as convenient
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
      logError("The constraints for joint '%s' are such that there are no possible values for the joint: min_bound: %g, max_bound: %g. Failing.\n", jm->getName().c_str(), ji.min_bound_, ji.max_bound_);
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

  // Load IK solvers and get leg positions ------------------------------------

  // Create publishers for rviz
  ros::NodeHandle nh_("~");
  robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>( "/hrp2_demos", 1 );

  left_leg_ = scene_->getRobotModel()->getJointModelGroup("left_leg");
  right_leg_ = scene_->getRobotModel()->getJointModelGroup("right_leg");

  // Create a default robot state so that we can record its foot positions
  moveit::core::RobotState goal_state(scene_->getRobotModel());
  goal_state.setToDefaultValues();

  // Get state of feet
  left_foot_position_ = goal_state.getGlobalLinkTransform("LLEG_LINK5");
  right_foot_position_ = goal_state.getGlobalLinkTransform("RLEG_LINK5");
  // ------------------------------------------------------------------------

  //logWarn("%s finished configuring", sampler_name_.c_str());
  return true;
}

bool HRP2JSKNTConstraintSampler::sample(robot_state::RobotState &state,
  const robot_state::RobotState & /* reference_state */,
  unsigned int max_attempts)
{
  if (!jmg_)
    logError("no joint model group loaded");

  if (!is_valid_)
  {
    logWarn("HRP2JSKNTConstraintSampler not configured, won't sample");
    return false;
  }

  logWarn("%s HRP2JSKNTConstraintSampler SAMPLING -----------------------------",sampler_name_.c_str());

  // Try 10 times
  for (std::size_t i = 0; i < 10; ++i)
  {
    logInform("Sampling attempt number %d", i);

    // Calculate random position of robot
    // \todo: don't sample virtual joint orientation and the legs to save time
    if (!sampleJoints(state))
    {
      logError("Unable to sample joints");
      return false;
    }

    // Sample the orientation of the virtual joint quaternion within bounds
    if (!sampleOrientationConstraints(state))
    {
      logError("Unable to sample orientation constraint");
      return false;
    }

    //printVirtualJointPosition(state);

    // show in rviz
    //displayRobotState(state);
    //ros::Duration(1.0).sleep();

    // Now do IK to find leg position
    if (calculateLegJoints(state, max_attempts))
    {
      // We found a valid location for both feet!

      // show in rviz
      //logInform("display robot state");
      //displayRobotState(state);
      //logInform("done display robot state");
      //ros::Duration(1.0).sleep();

      return true;
    }
  }
  return false;
}

bool HRP2JSKNTConstraintSampler::sampleJoints(robot_state::RobotState &state)
{
  // sample the unbounded joints first (in case some joint varipables are bounded)
  std::vector<double> v;
  for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
  {
    v.resize(unbounded_[i]->getVariableCount());

    if (false)
      logInform("%s UNCONSTRAINED: Joint number %d named %s with variables %d", sampler_name_.c_str(),
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
      logInform("%s CONSTRAINED: Joint number %d named %s bounds [%f,%f]", sampler_name_.c_str(), bounds_[i].index_,
        jmg_->getVariableNames()[ bounds_[i].index_ ].c_str(),
        bounds_[i].min_bound_, bounds_[i].max_bound_);

    values_[bounds_[i].index_] = random_number_generator_.uniformReal(bounds_[i].min_bound_, bounds_[i].max_bound_);
  }

  state.setJointGroupPositions(jmg_, values_);

  return true;
}

bool HRP2JSKNTConstraintSampler::sampleOrientationConstraints(robot_state::RobotState &state)
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
    const Eigen::Affine3d &t = state.getFrameTransform(orientation_constraint_->getReferenceFrame());
    Eigen::Affine3d rt(t.rotation() * quat.toRotationMatrix());
    quat = Eigen::Quaterniond(rt.rotation());
  }

  // Now set the virtual joint quaternion to this result
  state.setVariablePosition("virtual_joint/rot_x", quat.x());
  state.setVariablePosition("virtual_joint/rot_y", quat.y());
  state.setVariablePosition("virtual_joint/rot_z", quat.x());
  state.setVariablePosition("virtual_joint/rot_w", quat.w());

  return true;
}

bool HRP2JSKNTConstraintSampler::calculateLegJoints(robot_state::RobotState &state, unsigned int max_attempts)
{

  // Move feet positions to be directly under torso, in a regular parallel position, squatting if necessary
  const Eigen::Affine3d &base_link = state.getJointTransform("virtual_joint");
  const Eigen::Translation3d floor_z(0,0,-base_link.translation().z());

  // First move the standard foot positions to under the current virtual_joint
  left_foot_position_new_ = floor_z * base_link * left_foot_position_;
  // Then move the z-axis up level with the floor
  right_foot_position_new_ = floor_z * base_link * right_foot_position_;

  // solve for one leg
  if (state.setFromIK(left_leg_, left_foot_position_new_, max_attempts, 0.1))
  {
    //logInform("Found IK Solution for left!");

    // solve for other leg
    if (state.setFromIK(right_leg_, right_foot_position_new_, max_attempts, 0.1))
    {
      logInform("Found IK Solution for BOTH!");
    }
    else
    {
      logError("Did not find IK solution with %d attempts", max_attempts);
      return false;
    }
  }
  else
  {
    logError("Did not find IK solution with %d attempts", max_attempts);
    return false;
  }

  return true;
}

bool HRP2JSKNTConstraintSampler::project(robot_state::RobotState &state,
  unsigned int max_attempts)
{
  return sample(state, state, max_attempts);
}

void HRP2JSKNTConstraintSampler::clear()
{
  ConstraintSampler::clear();
  bounds_.clear();
  unbounded_.clear();
  uindex_.clear();
  values_.clear();
}

void HRP2JSKNTConstraintSampler::displayRobotState(const robot_state::RobotState &robot_state)
{
  // send the message to the RobotState display
  robot_state::robotStateToRobotStateMsg(robot_state, display_robot_msg_.state);
  robot_state_publisher_.publish( display_robot_msg_ );
  ros::spinOnce();
}

} //namespace
