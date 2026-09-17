#include "moveit_isaac_controller_manager/isaac_controller.h"

namespace moveit_isaac_controller_manager
{

IsaacController::IsaacController(
    const std::string& name,
    const ros::Publisher& pub)
  : MoveItControllerHandle(name)
  , trajectory_pub_(pub)
  , status_(moveit_controller_manager::ExecutionStatus::SUCCEEDED)
{
    ROS_INFO_STREAM("IsaacController created: " << name);
}

bool IsaacController::sendTrajectory(
    const moveit_msgs::RobotTrajectory& trajectory)
{
    ROS_INFO_STREAM(
        "Publishing trajectory with "
        << trajectory.joint_trajectory.points.size()
        << " points."
    );

    trajectory_pub_.publish(trajectory.joint_trajectory);

    status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;

    return true;
}

bool IsaacController::cancelExecution()
{
    ROS_WARN("Cancel execution requested.");

    status_ = moveit_controller_manager::ExecutionStatus::ABORTED;

    return true;
}

bool IsaacController::waitForExecution(
    const ros::Duration& timeout)
{
    if (timeout.toSec() > 0.0)
        timeout.sleep();

    return true;
}

moveit_controller_manager::ExecutionStatus
IsaacController::getLastExecutionStatus()
{
    return status_;
}

}