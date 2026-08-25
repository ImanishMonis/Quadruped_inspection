#pragma once

#include <moveit/controller_manager/controller_manager.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <ros/ros.h>

namespace moveit_isaac_controller_manager
{

class IsaacController :
    public moveit_controller_manager::MoveItControllerHandle
{
public:

    IsaacController(
        const std::string& name,
        const ros::Publisher& pub
    );

    bool sendTrajectory(
        const moveit_msgs::RobotTrajectory& trajectory
    ) override;

    bool cancelExecution() override;

    bool waitForExecution(
        const ros::Duration& timeout = ros::Duration(0)
    ) override;

    moveit_controller_manager::ExecutionStatus
    getLastExecutionStatus() override;

private:

    ros::Publisher trajectory_pub_;

    moveit_controller_manager::ExecutionStatus status_;
};

}