#pragma once

#include "simulation2d/lidar.h"

#include<eigen3/Eigen/Geometry>


class Robot
{
private:
    /**
     * @brief pose The pose of the robot.
     */
    Eigen::Affine2f pose;
    /**
     * @brief lidar Laserscanner of the robot.
     */
    Lidar lidar;
    /**
     * @brief collision_radius Collision radius of the robot.
     */
    float collision_radius;
    /**
     * @brief max_velocity Max velocity of the robot. Important for the minimum timestamp simulation step.
     */
    float max_velocity;
public:
    /**
     * @brief Robot Constructor of the robot class.
     */
    Robot();
    /**
     * @brief Robot Constructor of the robot class.
     * @param position Robot position as vector.
     * @param orientation Orientation in radiant.
     * @param collision_radius Collision radius.
     * @param max_velocity Maximal velocity of the robot.
     */
    Robot(Eigen::Vector2f position, float orientation, float collision_radius, float max_velocity);
    /**
     * @brief Robot Constructor of the robot class.
     * @param x X position.
     * @param y Y position.
     * @param orientation Orientation in radiant.
     * @param collision_radius Collision radius.
     * @param max_velocity Maximal velocity.
     */
    Robot(float x, float y, float orientation, float collision_radius, float max_velocity);

    /**
     * @brief get_pose Get the pose of the robot.
     * @return Pose.
     */
    Eigen::Affine2f get_pose() const;
    /**
     * @brief get_position Get the position of the robot as vector.
     * @return Vector.
     */
    Eigen::Vector2f get_position() const;
    /**
     * @brief get_position_x Get the x position of the robot.
     * @return X position.
     */
    float get_position_x() const;
    /**
     * @brief get_position_y Get the y position of the robot.
     * @return Y position.
     */
    float get_position_y() const;
    /**
     * @brief get_collision_radius Get the collision radius of the robot.
     * @return Collision radius.
     */
    float get_collision_radius() const;
    /**
     * @brief get_orientation Get the orientation of the robot in radiant.
     * @return Orientation in radiant.
     */
    float get_orientation() const;
    /**
     * @brief get_max_velocity Get the maximal velocity of the robot.
     * @return Maximal velocity.
     */
    float get_max_velocity() const;

    /**
     * @brief set_pose Set the pose of the robot.
     * @param pose Robot pose.
     */
    void set_pose(Eigen::Affine2f pose);
    /**
     * @brief set_pose Set the pose of the robot.
     * @param position Position as vector.
     * @param orientation Orientation in radiant.
     */
    void set_pose(Eigen::Vector2f position, float orientation);
    /**
     * @brief set_pose Set the pose of the rboto.
     * @param x X position of the robot.
     * @param y Y position of the robot.
     * @param orientation Orientation of the robot in radiant.
     */
    void set_pose(float x, float y, float orientation);
    /**
     * @brief set_position Set the position of the robot.
     * @param position Position as vector.
     */
    void set_position(Eigen::Vector2f position);
    /**
     * @brief set_position Set the position of the robot.
     * @param x X position.
     * @param y Y position.
     */
    void set_position(float x, float y);
    /**
     * @brief set_orientation Set the orientation of the robot.
     * @param orientation Orientation in radiant.
     */
    void set_orientation(float orientation);
    /**
     * @brief move Move the robot.
     * @param linear_velocity Linear velocity.
     * @param angular_velocity Angular velocity.
     */
    void move(float linear_velocity, float angular_velocity);
    /**
     * @brief get_lidar Get the lidar sensor of the robot.
     * @return Lidar sensor.
     */
    Lidar &get_lidar();
    /**
     * @brief get_lidar Get the lidar sensor of the robot.
     * @return Lidar sensor.
     */
    const Lidar &get_lidar() const;
};
