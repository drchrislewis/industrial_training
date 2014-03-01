/*
 * pickup_box.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: ros-industrial
 */


#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Use the 'move_group' object to set the wrist as the end-effector link
    - Use the 'move_group' object to set the world frame as the reference frame for path planning
    - Move the robot to each pick pose.
    - Close gripper after reaching the approach pose

  Hints:
    - The 'move_group' interface has useful methods such as 'setEndEffectorLink' and 'setPoseReferenceFrame' that
      can be used to prepare the robot for planning.
    - The 'setPoseTarget' method allows you to set a "pose" as your target to move the robot.
*/
void collision_avoidance_pick_and_place::PickAndPlace::pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose)
{
	  //ROS_ERROR_STREAM("move_through_pick_poses is not implemented yet.  Aborting."); exit(1);

	  // task variables
	  bool success;

	  // set the wrist as the end-effector link
	  //   - the robot will try to move this link to the specified position
	  //   - if not specified, MoveIt will use the last link in the arm group
	  /* Fill Code: [ use the 'setEndEffectorLink' in the 'move_group' object] */
	  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

	  // set world frame as the reference
	  //   - the target position is specified relative to this frame
	  //   - if not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
	  /* Fill Code: [ use the 'setPoseReferenceFrame' in the 'move_group' object] */
	  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {

	    ros::Duration(2.0f).sleep();
	    move_group_interface::MoveGroup::Plan plan;
	    success = create_motion_plan(pick_poses[i],plan) && move_group_ptr->execute(plan);

	    // verifying move completion
	    if(success)
	    {
	      ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
	    }
	    else
	    {
	      ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
	      exit(1);
	    }


	    if(i == 1)
	    {
		// turn on gripper suction after approach pose
		/* Fill Code: [ call the 'set_gripper' function to turn on suction ] */
	      set_gripper(true);
	    }

	    if(i == 2)
	    {
	    	// attach box to end effector
	    	set_attached_object(true,box_pose);
	    }

	  }

}

