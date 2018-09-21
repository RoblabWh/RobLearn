#include "simulation2d/robot.h"

Robot::Robot()
{
    this->pose.setIdentity();

    this->collision_radius = 0.175f;

    this->max_velocity = 1.0f;
}

Robot::Robot(Eigen::Vector2f position, float orientation, float collision_radius, float max_velocity)
{
    this->pose.setIdentity();
    this->pose.translate(position);
    this->pose.rotate(orientation);

    this->collision_radius = collision_radius;

    this->max_velocity = max_velocity;
}

Robot::Robot(float x, float y, float orientation, float collision_radius, float max_velocity)
{
    this->pose.setIdentity();
    this->pose.translate(Eigen::Vector2f(x,y));
    this->pose.rotate(orientation);

    this->collision_radius = collision_radius;

    this->max_velocity;
}

void Robot::set_pose(Eigen::Vector2f position, float orientation)
{
    this->pose.setIdentity();
    this->pose.translate(position);
    this->pose.rotate(orientation);
}

void Robot::set_pose(float x, float y, float orientation)
{
    this->pose.setIdentity();
    this->pose.translate(Eigen::Vector2f(x,y));
    this->pose.rotate(orientation);
}

void Robot::set_orientation(float orientation)
{
    this->set_pose(this->pose.translation(), orientation);
}

void Robot::set_position(Eigen::Vector2f position)
{
    this->set_pose(position, this->get_orientation());
}

void Robot::set_position(float x, float y)
{
    this->set_pose(Eigen::Vector2f(x,y), this->get_orientation());
}

void Robot::set_pose(Eigen::Affine2f pose)
{
    this->pose = pose;
}

Eigen::Affine2f Robot::get_pose() const
{
    return this->pose;
}

Eigen::Vector2f Robot::get_position() const
{
    return this->pose.translation();
}

float Robot::get_position_x() const
{
    return this->pose.translation().x();
}

float Robot::get_position_y() const
{
    return this->pose.translation().y();
}

float Robot::get_orientation() const
{
    return Eigen::Rotation2Df(this->pose.rotation()).angle();
}

float Robot::get_collision_radius() const
{
    return this->collision_radius;
}

void Robot::move(float linear_velocity, float angular_velocity)
{
    this->pose.translate(Eigen::Vector2f(linear_velocity, 0.0f));
    this->pose.rotate(angular_velocity);
}

float Robot::get_max_velocity() const
{
    return this->max_velocity;
}

Lidar &Robot::get_lidar()
{
    return this->lidar;
}

Lidar const &Robot::get_lidar() const
{
    return this->lidar;
}

