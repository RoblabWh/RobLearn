#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"

#include <eigen3/Eigen/Core>

class Visualization_Gnuplot
{
private:
    Eigen::Vector2f point_min;
    Eigen::Vector2f point_max;

    FILE *pipe;
public:
    Visualization_Gnuplot();
    ~Visualization_Gnuplot();
    void init(const DataContainer &data);
    void visualize(const Robot &robot, const DataContainer &data);
};
