#include <ros/ros.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <uibk_planning_node/KinematicsHelper.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "test_ccontrol"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    boost::shared_ptr<trajectory_planner_moveit::TrajectoryPlanner> plannerPtr;
    plannerPtr = boost::shared_ptr<trajectory_planner_moveit::TrajectoryPlanner>(new trajectory_planner_moveit::TrajectoryPlanner(*node));
    plannerPtr->setArm("right");
    plannerPtr->setPlannerId("LBKPIECEkConfigDefault");

    geometry_msgs::Pose goal1;
    moveit_msgs::MotionPlanResponse plan;

    goal1.position.x = 0.26;
    goal1.position.y = 0.20;
    goal1.position.z = 0.65;
    goal1.orientation.x = 0.755872;
    goal1.orientation.y = -0.612878;
    goal1.orientation.z = -0.0464803;
    goal1.orientation.w = 0.22556;

    if(plannerPtr->plan(goal1, plan)) {

        ROS_INFO("Plan to goal1 found");
        moveit_msgs::RobotState state;

        state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

        ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");

        plannerPtr->executePlan(plan, execution_client);

    } else {
        ROS_ERROR("Planning for goal1 failed!");
    }

    return EXIT_SUCCESS;
}

