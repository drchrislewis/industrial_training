/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionGoal.h>
#include <robot_io/DigitalOutputUpdate.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>

using namespace object_manipulation_msgs;
using namespace actionlib;


typedef robot_io::DigitalOutputUpdate::Request DigitalOutputType;

static const std::string OUTPUT_TOPIC = "/digital_outputs";
static const std::string INPUT_TOPIC = "/digital_inputs";
static const std::string OUTPUT_SERVICE = "/digital_output_update";

#define DEFAULT_VACCUM_VALVE 3
#define DEFAULT_FLACID_VALVE 1

class VersaballActionServer
{
private:
  typedef ActionServer<GraspHandPostureExecutionAction> GEAS;
  typedef GEAS::GoalHandle GoalHandle;

public:
  VersaballActionServer(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "grasp_execution_action",
                   boost::bind(&VersaballActionServer::goalCB, this, _1),
                   boost::bind(&VersaballActionServer::cancelCB, this, _1),
                   false),
    ball_rigid_output_channel_(DEFAULT_VACCUM_VALVE),
    ball_flacid_output_channel_(DEFAULT_FLACID_VALVE)
  {

  }

  ~VersaballActionServer()
  {
  }

  void init()
  {
    ros::NodeHandle pn("/");
    std::string nodeName = ros::this_node::getName();

    // service client
    service_client_ = pn.serviceClient<robot_io::DigitalOutputUpdate>(OUTPUT_SERVICE);
    while(!service_client_.waitForExistence(ros::Duration(5.0f)))
      {
	ROS_INFO_STREAM(nodeName<<": Waiting for "<<OUTPUT_SERVICE<<" to start");
      }

    if(!fetchParameters() )
      {
	ROS_ERROR_STREAM(nodeName<<": Did not find required ros parameters, exiting");
	ros::shutdown();
	return;
      }

    if(!validateChannelIndices())
      {
	ROS_ERROR_STREAM(nodeName<<": One or more parameter values are invalid");
	ros::shutdown();
	return;
      }

    action_server_.start();
    ROS_INFO_STREAM(nodeName<<": Grasp execution action node started");
  }

private:


  void goalCB(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();

    ROS_INFO("%s",(nodeName + ": Received grasping goal").c_str());

    robot_io::DigitalOutputUpdate::Request req;
    robot_io::DigitalOutputUpdate::Response res;

    switch(gh.getGoal()->goal)
      {
      case GraspHandPostureExecutionGoal::PRE_GRASP:

	gh.setAccepted();
	ROS_INFO_STREAM(nodeName + ": Pre-grasp command accepted");

	req.bit_index = ball_rigid_output_channel_;
	req.output_bit_state = true;
	if(service_client_.call(req,res))
	  {
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp insure vaccume off succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp command aborted");
	  }

	req.bit_index = ball_flacid_output_channel_;
	req.output_bit_state = true;
	if(service_client_.call(req,res))
	  {
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp inflate command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp command aborted");
	  }

	sleep(2);

	req.bit_index = ball_flacid_output_channel_;
	req.output_bit_state = false;

	if(service_client_.call(req,res))
	  {
	    gh.setSucceeded();
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp stop inflating command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp command aborted");
	  }
	break;

      case GraspHandPostureExecutionGoal::GRASP:

	gh.setAccepted();
	ROS_INFO_STREAM(nodeName + ": Grasp command accepted");

	req.bit_index = ball_flacid_output_channel_;
	req.output_bit_state = false;

	if(service_client_.call(req,res))
	  {
	    ROS_INFO_STREAM(nodeName + ": Grasp: insure not inflating succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Pre-grasp command aborted");
	  }

	req.bit_index = ball_rigid_output_channel_;
	req.output_bit_state = true;
	if(service_client_.call(req,res))
	  {
	    gh.setSucceeded();
	    ROS_INFO_STREAM(nodeName + ": Grasp command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Grasp command aborted");
	    break;
	  }

	gh.setSucceeded();
	ROS_INFO_STREAM(nodeName + ": Grasp command succeeded");
	break;

      case GraspHandPostureExecutionGoal::RELEASE:

	gh.setAccepted();
	ROS_INFO_STREAM(nodeName + ": Release command accepted");

	req.bit_index = ball_rigid_output_channel_;
	req.output_bit_state = false;
	if(service_client_.call(req,res))
	  {
	    ROS_INFO_STREAM(nodeName + ": turn off ball rigid command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Release aborted, failed release of ball rigid valve");
	  }
	req.bit_index = ball_flacid_output_channel_;
	req.output_bit_state = true;
	if(service_client_.call(req,res))
	  {
	    ROS_INFO_STREAM(nodeName + ": Release command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Release aborted, failed release of ball flacid valve");
	  }

        sleep(2);

	req.bit_index = ball_flacid_output_channel_;
	req.output_bit_state = false;
	if(service_client_.call(req,res))
	  {
	    gh.setSucceeded();
	    ROS_INFO_STREAM(nodeName + ": Release command succeeded");
	  }
	else
	  {
	    gh.setAborted();
	    ROS_INFO_STREAM(nodeName + ": Release aborted, failed release of ball flacid valve");
	  }

	break;

      default:

	ROS_WARN_STREAM(nodeName + ": Unidentified grasp request, rejecting request");
	gh.setRejected();
	break;
      }

  }

  void cancelCB(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();
    ROS_INFO_STREAM(nodeName + ": Canceling current grasp action");
    gh.setCanceled();
    ROS_INFO_STREAM(nodeName + ": Current grasp action has been canceled");
  }

  bool fetchParameters()
  {
    ros::NodeHandle nh;

    bool success = true;
    success = success && nh.getParam("ball_rigid_channel",ball_rigid_output_channel_);
    success = success && nh.getParam("ball_flacid_channel",ball_flacid_output_channel_);
    return success;
  }

  bool validateChannelIndices()
  {
    typedef robot_io::DigitalOutputUpdate::Request DigitalOutputType;

    if(ball_rigid_output_channel_ >= (int)DigitalOutputType::COUNT )
      {
	return false;
      }
    if( ball_rigid_output_channel_ == (int)DigitalOutputType::COLLISION)
      {
	return false;
      }
    if(ball_flacid_output_channel_ >= (int)DigitalOutputType::COUNT)
      {
	return false;
      }
    if( ball_flacid_output_channel_ == (int)DigitalOutputType::COLLISION)
      {
	return false;
      }
    if(ball_rigid_output_channel_ == ball_flacid_output_channel_)
      {
	return false;
      }
    return true;
  }

  // ros comm
  ros::NodeHandle node_;
  GEAS action_server_;
  ros::ServiceClient service_client_;

  // ros parameters
  int ball_rigid_output_channel_; // index to valve controlling main vaccum
  int ball_flacid_output_channel_; // index to valve controlling release of vaccum

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_execution_action_node");
  ros::NodeHandle node("");
  VersaballActionServer ge(node);
  ge.init();
  ros::spin();
  return 0;
}




