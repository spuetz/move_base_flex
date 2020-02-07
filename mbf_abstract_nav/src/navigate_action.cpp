/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  navigate_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/navigate_action.h"

namespace mbf_abstract_nav
{

NavigateAction::NavigateAction(const std::string &name,
                               const RobotInformation &robot_info)
  :  name_(name), robot_info_(robot_info), private_nh_("~"),
     action_client_exe_path_(private_nh_, "exe_path"),
     action_client_spin_turn_("spin_turn"),
     oscillation_timeout_(0),
     oscillation_distance_(0),
     action_state_(IDLE),
     replanning_(false),
     replanning_rate_(1.0)
{
}

NavigateAction::~NavigateAction()
{
}

void NavigateAction::reconfigure(
    mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  oscillation_timeout_ = ros::Duration(config.oscillation_timeout);
  oscillation_distance_ = config.oscillation_distance;
}

void NavigateAction::cancel()
{
  action_state_ = CANCELED;

  ROS_INFO_STREAM_NAMED ("navigate", "Cancel called for navigate");
  goal_handle_.setCanceled();

  if(!action_client_exe_path_.getState().isDone())
  {
    action_client_exe_path_.cancelGoal();
  }

  if(!action_client_spin_turn_.getState().isDone())
  {
    action_client_spin_turn_.cancelGoal();
  }
  

}

void NavigateAction::start(GoalHandle &goal_handle)
{
  action_state_ = SPLIT_PATH;

  goal_handle.setAccepted();

  goal_handle_ = goal_handle;

  ROS_DEBUG_STREAM_NAMED("navigate", "Start action "  << "navigate");

  const forklift_interfaces::NavigateGoal& goal = *(goal_handle.getGoal().get());
  const forklift_interfaces::NavigatePath &plan = goal.path;

  forklift_interfaces::NavigateResult navigate_result;

  exe_path_goal_.controller = goal.controller;

  ros::Duration connection_timeout(1.0);

  last_oscillation_reset_ = ros::Time::now();

  path_segments_.clear();


  geometry_msgs::PoseStamped robot_pose;
  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!robot_info_.getRobotPose(robot_pose))
  {
    ROS_ERROR_STREAM_NAMED("navigate", "Could not get the current robot pose!");
    navigate_result.remarks = "Could not get the current robot pose!";
    navigate_result.status = forklift_interfaces::NavigateResult::TF_ERROR;
    goal_handle.setAborted(navigate_result, navigate_result.remarks);
    return;
  }

  // wait for server connections
  if (!action_client_exe_path_.waitForServer(connection_timeout) ||
      !action_client_spin_turn_.waitForServer(connection_timeout))
  {
    ROS_ERROR_STREAM_NAMED("navigate", "Could not connect to one or more of navigate actions: exe_path, spin_turn!");
    navigate_result.status = forklift_interfaces::NavigateResult::INTERNAL_ERROR;
    navigate_result.remarks = "Could not connect to the navigate actions!";
    goal_handle.setAborted(navigate_result, navigate_result.remarks);
    return;
  }

  // call get_path action server to get a first plan

  bool split_result;
  split_result = getSplitPath(plan, path_segments_);

  if(!split_result)
  {
    ROS_ERROR_STREAM_NAMED("navigate", "Path provided was empty!");
    navigate_result.remarks = "Empty path provided!";
    navigate_result.status = forklift_interfaces::NavigateResult::INVALID_PATH;
    goal_handle.setAborted(navigate_result, navigate_result.remarks);
    return;
  }

  startNavigate();


  
}

void NavigateAction::startNavigate()
{
  action_state_ = NAVIGATE;
  while (action_state_ != SUCCEEDED || action_state_ != FAILED)
  {
    switch (action_state_)
    {
    case NAVIGATE:
      runNavigate();
      break;
    case EXE_PATH:
      action_client_exe_path_.sendGoal(
          exe_path_goal_,
          boost::bind(&NavigateAction::actionExePathDone, this, _1, _2),
          boost::bind(&NavigateAction::actionExePathActive, this),
          boost::bind(&NavigateAction::actionExePathFeedback, this, _1));
      action_state_ = ACTIVE;
      break;
    case SPIN_TURN:
      action_client_spin_turn_.sendGoal(
        spin_turn_goal_,
        boost::bind(&NavigateAction::actionSpinTurnDone, this, _1, _2),
        boost::bind(&NavigateAction::actionSpinTurnActive, this));
      action_state_ = ACTIVE;
      break;
    case OSCILLATING:
      /* code */
      break; 
    
    default:
      break;
    }
    ros::spinOnce();
  }
}

void NavigateAction::runNavigate()
{
  action_state_ = FAILED;
  ROS_INFO_STREAM_NAMED("navigate", "Segments remaning: " << path_segments_.size());
  if(!path_segments_.empty())
  {
    ROS_INFO_STREAM_NAMED("navigate","Spin turn: "<< static_cast<int>(path_segments_.front().checkpoints.front().spin_turn));
    if(path_segments_.front().checkpoints.front().spin_turn)
    {
      const auto orientation = path_segments_.front().checkpoints.front().pose.pose.orientation;
      geometry_msgs::PoseStamped robot_pose;
      robot_info_.getRobotPose(robot_pose);
      double min_angle = mbf_utility::angle(robot_pose , path_segments_.front().checkpoints.front().pose);
      double yaw_goal = getSpinAngle(orientation);
      ROS_INFO_STREAM("min_angle: " << min_angle);
      if (fabs(min_angle)<10.0)
      {
        path_segments_.front().checkpoints.front().spin_turn = false;
        action_state_ = NAVIGATE;
        return;
      }

      spin_turn_goal_.angle = yaw_goal;
      action_state_ = SPIN_TURN;
      path_segments_.front().checkpoints.front().spin_turn = false;
      return;
    }
    else
    {
      for (const auto& point : path_segments_.front().checkpoints)
      {
          ROS_INFO_STREAM_NAMED("navigate","["<< point.pose.pose.position.x << "," << point.pose.pose.position.y << "]");
      }
      exe_path_goal_.path = path_segments_.front();

      action_state_ = EXE_PATH;
      return;
    }
  }
  else
  {
    forklift_interfaces::NavigateResult navigate_result;
    navigate_result.status = forklift_interfaces::NavigateResult::SUCCESS;
    navigate_result.remarks = "Navigate action completed successfully!";
    navigate_result.final_pose = robot_pose_;
    ROS_INFO_STREAM_NAMED("navigate", "Navigate action completed successfully");
    goal_handle_.setSucceeded(navigate_result, navigate_result.remarks);
    
    action_state_ = SUCCEEDED;
    return;
  }
    
}

void NavigateAction::actionSpinTurnActive()
{
  ROS_INFO_STREAM_NAMED("navigate", "The \"spin_turn\" action is active.");
  action_state_ = ACTIVE;
}


void NavigateAction::actionExePathActive()
{
  ROS_DEBUG_STREAM_NAMED("navigate", "The \"exe_path\" action is active.");
  action_state_= ACTIVE;
}

void NavigateAction::actionExePathFeedback(
    const mbf_msgs::ExePathFeedbackConstPtr &feedback)
{
  navigate_feedback_.status = feedback->status;
  navigate_feedback_.remarks = feedback->remarks;
  navigate_feedback_.angle_to_goal = feedback->angle_to_goal;
  navigate_feedback_.dist_to_goal = feedback->dist_to_goal;
  navigate_feedback_.current_pose = feedback->current_pose;
  navigate_feedback_.velocity = feedback->velocity;
  navigate_feedback_.last_checkpoint = feedback->last_checkpoint;
  navigate_feedback_.target_checkpoint = feedback->target_checkpoint;
  robot_pose_ = feedback->current_pose;
  goal_handle_.publishFeedback(navigate_feedback_);

  // we create a navigation-level oscillation detection using exe_path action's feedback,
  // as the later doesn't handle oscillations created by quickly failing repeated plans

  // if oscillation detection is enabled by osciallation_timeout != 0
  if (!oscillation_timeout_.isZero())
  {
    // check if oscillating
    // moved more than the minimum oscillation distance
    if (mbf_utility::distance(robot_pose_, last_oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      last_oscillation_pose_ = robot_pose_;
    }
    else if (last_oscillation_reset_ + oscillation_timeout_ < ros::Time::now())
    {
      std::stringstream oscillation_msgs;
      oscillation_msgs << "Robot is oscillating for " << (ros::Time::now() - last_oscillation_reset_).toSec() << "s!";
      ROS_WARN_STREAM_NAMED("exe_path", oscillation_msgs.str());
      action_client_exe_path_.cancelGoal();


      forklift_interfaces::NavigateResult navigate_result;
      navigate_result.status = forklift_interfaces::NavigateResult::OSCILLATION;
      navigate_result.remarks = oscillation_msgs.str();
      navigate_result.final_pose = robot_pose_;
      navigate_result.angle_to_goal = navigate_feedback_.angle_to_goal;
      navigate_result.dist_to_goal = navigate_feedback_.dist_to_goal;
      goal_handle_.setAborted(navigate_result, navigate_result.remarks);

    }
  }
}

bool NavigateAction::getSplitPath(
      const forklift_interfaces::NavigatePath &plan,
      std::vector<forklift_interfaces::NavigatePath> &result)
{
  ROS_INFO_STREAM_NAMED("navigate","Splitting the path");
  if(plan.checkpoints.size()<1)
  {
    return false;
  }
  forklift_interfaces::NavigatePath segment;

  for (size_t i = 0 ; i < plan.checkpoints.size(); i++)
  {
    
    segment.header = plan.header;
    segment.xy_goal_tolerance = plan.xy_goal_tolerance;
    segment.yaw_goal_tolerance = plan.xy_goal_tolerance;

    if (i<1)
    {
      segment.checkpoints.push_back(plan.checkpoints[i]);
    }
    else if (i<plan.checkpoints.size()-1)
    {
      if(!plan.checkpoints[i].spin_turn)
      {
        segment.checkpoints.push_back(plan.checkpoints[i]);
      }
      else
      {
        segment.checkpoints.push_back(plan.checkpoints[i]);
        result.push_back(segment);
        segment.checkpoints.clear();
        forklift_interfaces::Checkpoint checkpoint = plan.checkpoints[i];
        checkpoint.spin_turn = false;
        segment.checkpoints.push_back(checkpoint);
      }
    }
    else
    {
      segment.checkpoints.push_back(plan.checkpoints[i]);
      result.push_back(segment);
      segment.checkpoints.clear();
    }
  }

  for(const auto& segment : result)
  {
    ROS_INFO_STREAM_NAMED("navigate","Split segments:");
    for (const auto& point : segment.checkpoints)
    {
        ROS_INFO_STREAM_NAMED("navigate","["<< point.pose.pose.position.x << "," << point.pose.pose.position.y << "]" << "spin turn:" << static_cast<int>(point.spin_turn));
    }
  }

  return true;
}

void NavigateAction::actionSpinTurnDone(
    const actionlib::SimpleClientGoalState &state,
    const aifl_msg::SpinTurnResultConstPtr &result_ptr)
{
  action_state_ =  FAILED;

  ROS_INFO_STREAM_NAMED("navigate", "Action \"spin_turn\" finished.");

  const aifl_msg::SpinTurnResult& result = *(result_ptr.get());

  if(!(state == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ROS_INFO_STREAM_NAMED("navigate", "Action \"spin_turn\" did not succeed, retrying...");
    action_state_ = SPIN_TURN;
  }
  else
  {
    if(result.status != 2)
    {
      ROS_ERROR_STREAM_NAMED("navigate", "Action \"spin_turn\" failed :" << result);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("navigate", "Action \"spin_turn\" completed successfully");
      action_state_ = NAVIGATE;
    }
    
  }

}

void NavigateAction::actionExePathDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::ExePathResultConstPtr &result_ptr)
{
  action_state_ =  FAILED;

  ROS_DEBUG_STREAM_NAMED("navigate", "Action \"exe_path\" finished.");

  const mbf_msgs::ExePathResult& result = *(result_ptr.get());
  const forklift_interfaces::NavigateGoal& goal = *(goal_handle_.getGoal().get());
  forklift_interfaces::NavigateResult navigate_result;

  navigate_result.status = result.status;
  navigate_result.remarks = result.remarks;
  navigate_result.dist_to_goal = result.dist_to_goal;
  navigate_result.angle_to_goal = result.angle_to_goal;
  navigate_result.final_pose = result.final_pose;

  ROS_DEBUG_STREAM_NAMED("exe_path", "Current state:" << state.toString());

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      if(path_segments_.front().checkpoints.back().spin_turn)
      {
        const auto orientation = path_segments_.front().checkpoints.front().pose.pose.orientation;        
        double yaw = getSpinAngle(orientation);
        spin_turn_goal_.angle = yaw;
        action_state_ = SPIN_TURN; 
        path_segments_.erase(path_segments_.begin());
      }
      else
      {
        path_segments_.erase(path_segments_.begin());
        action_state_ = NAVIGATE;
      }

      break;

    case actionlib::SimpleClientGoalState::ABORTED:
      switch (result.status)
      {
        case mbf_msgs::ExePathResult::INVALID_PATH:
        case mbf_msgs::ExePathResult::TF_ERROR:
        case mbf_msgs::ExePathResult::NOT_INITIALIZED:
        case mbf_msgs::ExePathResult::INVALID_PLUGIN:
        case mbf_msgs::ExePathResult::INTERNAL_ERROR:
          // none of these errors is recoverable
          goal_handle_.setAborted(navigate_result, state.getText());
          break;

        default:
          // all the rest are, so we start calling the recovery behaviors in sequence

          ROS_WARN_STREAM_NAMED("navigate", "Abort the execution of the controller: " << result.remarks);
          goal_handle_.setAborted(navigate_result, state.getText());
          
      }
      break;

    case actionlib::SimpleClientGoalState::PREEMPTED:
      // action was preempted successfully!
      ROS_DEBUG_STREAM_NAMED("navigate", "The action \""
          << "exe_path" << "\" was preempted successfully!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
      ROS_DEBUG_STREAM_NAMED("navigate", "The action \""
          << "exe_path" << "\" was recalled!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_DEBUG_STREAM_NAMED("navigate", "The action \""
          << "exe_path" << "\" was rejected!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::LOST:
      // TODO
      break;

    default:
      ROS_FATAL_STREAM_NAMED("navigate",
                             "Reached unreachable case! Unknown SimpleActionServer state!");
      goal_handle_.setAborted();
      break;
  }
}

double NavigateAction::getSpinAngle(geometry_msgs::Quaternion orientation)
{
    tf2::Quaternion qp(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 mp(qp);
    double roll, pitch, yaw;
    mp.getRPY(roll, pitch, yaw);

    yaw = yaw * 180 / M_PI;

    return yaw;
}



} /* namespace mbf_abstract_nav */

