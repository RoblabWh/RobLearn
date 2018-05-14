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
    float time_step_end;
    int skip_laser;

    bool collision;

    void calculate_iteration();

public:
    Simulation2D();
    bool init();
    void step(const float linear_velocity, const float angular_velocity);
    void test();
    Eigen::Affine2f get_robot_pose();
    void set_robot_pose(const float x, const float y, const float orientation);
    void visualize();
};
