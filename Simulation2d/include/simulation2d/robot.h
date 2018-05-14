#pragma once

#include "simulation2d/lidar.h"

#include<eigen3/Eigen/Geometry>


class Robot
{
private:
    Eigen::Affine2f pose;
    Lidar lidar;
    float collision_radius;
    float max_velocity;
public:
    Robot();
    Robot(Eigen::Vector2f position, float orientation, float collision_radius, float max_velocity);
    Robot(float x, float y, float orientation, float collision_radius, float max_velocity);

    Eigen::Affine2f get_pose() const;
    Eigen::Vector2f get_position() const;
    float get_position_x() const;
    float get_position_y() const;
    float get_collision_radius() const;
    float get_orientation() const;
    float get_max_velocity() const;

    void set_pose(Eigen::Affine2f pose);
    void set_pose(Eigen::Vector2f position, float orientation);
    void set_pose(float x, float y, float orientation);
    void set_position(Eigen::Vector2f position);
    void set_position(float x, float y);
    void set_orientation(float orientation);
    void move(float linear_velocity, float angular_velocity);
    Lidar &get_lidar();
    const Lidar &get_lidar() const;
};
