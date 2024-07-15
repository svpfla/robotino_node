#include "RobotState.h"
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

bool RobotState::loadURDF(const std::string &urdf_file_name)
{
    urdf::Model model;
    std::string package_path = ros::package::getPath("robotino_description");
    std::string urdf_path = package_path + "/urdf/" + urdf_file_name;

    if (!model.initFile(urdf_path))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    /**
     * TODO: Retrieve the wheel separation distance from the URDF file
     */

    /**
     * TODO: Retrieve the wheel radius from the URDF file
     */

    return true;
}

void RobotState::update(std::vector<float> motor_velocities, std::vector<int> motor_positions)
{
    /**
     * TODO: Implement the update function
     */

    int left_motor_position = motor_positions[0];
    int right_motor_position = motor_positions[2];

    int delta_left = left_motor_position - angle_left;
    int delta_right = right_motor_position - angle_right;

    float deltaS = (delta_left + delta_right) / 2.0 * wheel_radius_;
    float deltaPhi = (delta_right - delta_left) / wheel_distance_;

    float R = deltaS / deltaPhi;
    x += R * (sin(phi + deltaPhi) - sin(phi));
    y += -R * (cos(phi + deltaPhi) - cos(phi));
    phi += deltaPhi;

    angle_left = left_motor_position;
    angle_right = right_motor_position;

    sequence++;
}