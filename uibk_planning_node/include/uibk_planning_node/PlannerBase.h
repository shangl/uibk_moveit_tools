#ifndef PLANNERBASE_H
#define PLANNERBASE_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace trajectory_planner_moveit {


class PlannerBase {

protected:

    std::string arm_;

public:

    virtual ~PlannerBase() {}

    virtual const std::string getName() = 0;
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution) = 0;
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state) = 0;

    void setArm(const std::string &arm) { arm_ = arm; }
    std::string getArm() { return arm_; }

};

}

#endif // PLANNERBASE_H
