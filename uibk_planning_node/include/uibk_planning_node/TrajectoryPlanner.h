#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetMotionPlan.h>

#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
#include <moveit/move_group_interface/move_group_interface.h>
#else //ROS Indigo and before
#include <moveit/move_group_interface/move_group.h>
#endif


#include "../../src/PlannerBase.h"
#include "../../src/KinematicsHelper.h"

#define CAN_LOOK false
#define ALLOW_REPLAN false
#define UIBK_STD_GROUP_NAME "plan_kinematic_path"

namespace trajectory_planner_moveit {

class TrajectoryPlanner : public PlannerBase {

private:

    int max_traj_pts_;
    int planning_attempts_;

	double planning_time_;
    double goal_joint_tolerance_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;

  	std::string planner_id_;

    std::vector<std::string> jointNames;

    KinematicsHelper kin_helper_;

#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
    moveit::planning_interface::MoveGroupInterface _group;
#else   // ROS Indigo
    moveit::planning_interface::MoveGroup _group;
#endif

 	  ros::ServiceClient planning_client_;

    ros::NodeHandle nh;

public:

#if ROS_VERSION_MINIMUM(1, 12, 0) // ROS KINETIC
    TrajectoryPlanner(ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &group, const std::vector<std::string> jointNames, std::string kinematicPathTopic = UIBK_STD_GROUP_NAME);
#else   // ROS Indigo
    TrajectoryPlanner(ros::NodeHandle &nh, moveit::planning_interface::MoveGroup &group, const std::vector<std::string> jointNames, std::string kinematicPathTopic = UIBK_STD_GROUP_NAME);
#endif

	  ~TrajectoryPlanner();

    void setPlannerId(const std::string &planner_id);
    std::string getPlannerId();

    void setAllowedPlanningTime(double value);
    double getAllowedPlanningTime();

    void setPlanningAttempts(int value);
    int getPlanningAttempts();

    void setMaxTrajectoryPoints(int value);
    int getMaxTrajectoryPoints();

    const std::string getName();

    ros::ServiceClient execution_client;

    virtual bool plan(std::vector<double> &jointPos, moveit_msgs::MotionPlanResponse &solution);
    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution);
    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);

    virtual bool executePlan(moveit_msgs::RobotTrajectory& trajectory);
    virtual bool executePlan(moveit_msgs::MotionPlanResponse& trajectory);
    virtual bool executePlan(moveit_msgs::RobotTrajectory& trajectory, ros::ServiceClient& execution_client);
    virtual bool executePlan(moveit_msgs::MotionPlanResponse& trajectory, ros::ServiceClient& execution_client);

};


}


#endif // TRAJECTORYPLANNER_H
