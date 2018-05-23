#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"
#include "simulation2d/visualization_gnuplot.h"



class Simulation2D
{
private:
    DataContainer data;
    Robot robot;
    Visualization_Gnuplot visualization;


    float time_step_check_collision;
    float time_step_check_laser;

    bool collision;

    void calculate_iteration();
    void load_default_world();

public:
    Simulation2D();
    bool init(std::string world);
    void step(const float linear_velocity, const float angular_velocity, int skip_number=1);
    Eigen::Affine2f get_robot_pose();
    void set_robot_pose(const float x, const float y, const float orientation);
    void visualize();


    inline int observation_size() const
    {
        return robot.get_lidar().get_laser_size();
    }

    inline float observation_at(const int index) const
    {
        return robot.get_lidar().get_laser_distance_normalized(index);
    }

    inline float get_robot_pose_x() const
    {
        return robot.get_position_x();
    }

    inline float get_robot_pose_y() const
    {
        return robot.get_position_y();
    }

    inline float get_robot_pose_orientation() const
    {
        return robot.get_orientation();
    }

    inline bool done() const
    {
        return this->collision;
    }

    inline float observation_min_clustered_at(int index, const int cluster_size) const
    {
        float value = 1.0f;
        index *= cluster_size;
        int index_end = std::min(index + cluster_size, robot.get_lidar().get_laser_size());

        while(index < index_end)
        {
            value = std::min(value, robot.get_lidar().get_laser_distance_normalized(index));
            index++;
        }

        return value;
    }

    inline int observation_min_clustered_size(const int cluster_size) const
    {
        return robot.get_lidar().get_laser_size() / cluster_size;
    }

};


