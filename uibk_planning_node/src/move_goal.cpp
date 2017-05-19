/*
 * move_goal.cpp
 *
 *  Created on: Nov 30, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
#include <moveit/move_group_interface/move_group_interface.h>
#else //ROS Indigo and before
#include <moveit/move_group_interface/move_group.h>
#endif

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace ros;

/**
 * Moves the end effector to given pose.
 * Takes x, y, z and roll, pitch and yaw values as parameters.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

	if(argc < 7) {
		ROS_ERROR("Usage: %s x y z roll pitch yaw [optional ARM_NAME]", argv[0]);
		return EXIT_FAILURE;
	}

	ros::init(argc, argv, "move_goal");
	// this is necessary, because otherwise the calls to the movegroup interface will
	// not return, as they use service calls.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	double x,y,z,roll,pitch,yaw;

	x = atof(argv[1]);
	y = atof(argv[2]);
	z = atof(argv[3]);
	roll = atof(argv[4]);
	pitch = atof(argv[5]);
	yaw = atof(argv[6]);

	Eigen::Affine3d pose;

	pose = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

	pose.translation() = Eigen::Vector3d(x, y, z);

	// try to extract the planning group name from parameters
	// use 'rightArm' as default
	// possible values 'rightArm' and 'leftArm'
	string planning_group_name = "right_arm";
	if(argc > 7) {
		planning_group_name = argv[7];
	}

	ROS_INFO("Connecting to planning group '%s'", planning_group_name.c_str());

	// Create MoveGroup for one of the planning groups
#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
        moveit::planning_interface::MoveGroupInterface move_group(planning_group_name);
#else   // ROS Indigo
	move_group_interface::MoveGroup move_group(planning_group_name);
#endif
	move_group.setPlanningTime(5.0);
	move_group.setPoseReferenceFrame("world_link");



	ROS_INFO("Setting pose target");

	if(!move_group.setPoseTarget(pose)) {
		ROS_ERROR("Setting pose target failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Calling planning service");
#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
	moveit::planning_interface::MoveGroupInterface::Plan plan;
#else   // ROS Indigo
	move_group_interface::MoveGroup::Plan plan;
#endif
	if(!move_group.plan(plan)) {
		ROS_ERROR("Planning failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Motion plan calculated - executing trajectory.");

	if(!move_group.execute(plan)) {
		ROS_ERROR("Trajectory execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Pose target reached");

	geometry_msgs::PoseStamped pose_msg = move_group.getCurrentPose();
	ROS_INFO("Calculated position (frame_id = %s): x=%.2f, y=%.2f, z=%.2f", pose_msg.header.frame_id.c_str(), pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

	return EXIT_SUCCESS;

}
