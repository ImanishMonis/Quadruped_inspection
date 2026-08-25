#include "moveit_isaac_controller_manager/isaac_controller_manager.h"

#include <pluginlib/class_list_macros.hpp>

#include <trajectory_msgs/JointTrajectory.h>

#include <XmlRpcValue.h>

namespace moveit_isaac_controller_manager
{
    IsaacControllerManager::IsaacControllerManager()
    : nh_("~")
{
    trajectory_pub_ =
        nh_.advertise<trajectory_msgs::JointTrajectory>(
            "/joint_trajectory",
            1
        );

    if (!nh_.hasParam("controller_list"))
    {
        ROS_ERROR("No controller_list found.");

        return;
    }

    XmlRpc::XmlRpcValue controller_list;

    nh_.getParam(
        "controller_list",
        controller_list
    );

    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("controller_list must be an array.");
        return;
    }

    for (int i = 0; i < controller_list.size(); i++)
    {
        std::string name =
            std::string(controller_list[i]["name"]);

        std::vector<std::string> joints;

        for (int j = 0;
             j < controller_list[i]["joints"].size();
             j++)
        {
            joints.push_back(
                std::string(
                    controller_list[i]["joints"][j]
                )
            );
        }

        controllers_[name] =
            std::make_shared<IsaacController>(
                name,
                trajectory_pub_
            );

        controller_joints_[name] = joints;

        ControllerState state;

        state.active_ = true;

        state.default_ =
            controller_list[i].hasMember("default")
            ? bool(controller_list[i]["default"])
            : false;

        controller_states_[name] = state;

        ROS_INFO_STREAM(
            "Loaded controller: "
            << name
        );
    }
}

moveit_controller_manager::MoveItControllerHandlePtr
IsaacControllerManager::getControllerHandle(
    const std::string& name)
{
    auto it = controllers_.find(name);

    if (it != controllers_.end())
        return it->second;

    ROS_ERROR_STREAM("Unknown controller: " << name);

    return nullptr;
}


void IsaacControllerManager::getControllersList(
    std::vector<std::string>& names)
{
    names.clear();

    for (const auto& controller : controllers_)
        names.push_back(controller.first);
}


void IsaacControllerManager::getActiveControllers(
    std::vector<std::string>& names)
{
    getControllersList(names);
}

void IsaacControllerManager::getControllerJoints(
    const std::string& name,
    std::vector<std::string>& joints)
{
    auto it = controller_joints_.find(name);

    if (it != controller_joints_.end())
        joints = it->second;
    else
        joints.clear();
}


moveit_controller_manager::MoveItControllerManager::ControllerState
IsaacControllerManager::getControllerState(
    const std::string& name)
{
    return controller_states_[name];
}

bool IsaacControllerManager::switchControllers(
    const std::vector<std::string>&,
    const std::vector<std::string>&)
{
    return true;
}


} // namespace moveit_isaac_controller_manager

PLUGINLIB_EXPORT_CLASS(
    moveit_isaac_controller_manager::IsaacControllerManager,
    moveit_controller_manager::MoveItControllerManager
)