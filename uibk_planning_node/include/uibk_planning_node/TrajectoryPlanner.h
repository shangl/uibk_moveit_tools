#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <uibk_planning_node/KinematicsHelper.h>
#include <uibk_planning_node/PlannerBase.h>


#define CAN_LOOK false
#define ALLOW_REPLAN false
#define FRAME_ID "world_link"

namespace trajectory_planner_moveit {

class TrajectoryPlanner : public PlannerBase {

private:

	double planning_time_;
	int planning_attempts_;
	std::string planner_id_;
	int max_traj_pts_;
	double goal_joint_tolerance_;
	double goal_position_tolerance_;
	double goal_orientation_tolerance_;

	ros::ServiceClient planning_client_;
	KinematicsHelper kin_helper_;

    ros::NodeHandle nh;

public:

	TrajectoryPlanner(ros::NodeHandle &nh);

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

    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution);
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);

    virtual bool executePlan(moveit_msgs::RobotTrajectory& trajectory);
    virtual bool executePlan(moveit_msgs::MotionPlanResponse& trajectory);
    virtual bool executePlan(moveit_msgs::RobotTrajectory& trajectory, ros::ServiceClient& execution_client);
    virtual bool executePlan(moveit_msgs::MotionPlanResponse& trajectory, ros::ServiceClient& execution_client);

};


}


#endif // TRAJECTORYPLANNER_H
