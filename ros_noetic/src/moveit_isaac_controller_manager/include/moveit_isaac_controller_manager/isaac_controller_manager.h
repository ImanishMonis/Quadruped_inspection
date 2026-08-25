#pragma once

#include <moveit/controller_manager/controller_manager.h>

#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "moveit_isaac_controller_manager/isaac_controller.h"

namespace moveit_isaac_controller_manager
{

class IsaacControllerManager
    : public moveit_controller_manager::MoveItControllerManager
{
public:

    IsaacControllerManager();

    ~IsaacControllerManager() override = default;

    moveit_controller_manager::MoveItControllerHandlePtr
    getControllerHandle(
        const std::string& name
    ) override;

    void getControllersList(
        std::vector<std::string>& names
    ) override;

    void getActiveControllers(
        std::vector<std::string>& names
    ) override;

    void getControllerJoints(
        const std::string& name,
        std::vector<std::string>& joints
    ) override;

    ControllerState getControllerState(
        const std::string& name
    ) override;

    bool switchControllers(
        const std::vector<std::string>& activate,
        const std::vector<std::string>& deactivate
    ) override;

private:

    ros::NodeHandle nh_;

    ros::Publisher trajectory_pub_;

    std::map<std::string, std::shared_ptr<IsaacController>> controllers_;

    std::map<
        std::string,
        ControllerState
    > controller_states_;

    std::map<
        std::string,
        std::vector<std::string>
    > controller_joints_;
};

}