#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"

#include <eigen3/Eigen/Core>

class Visualization_Gnuplot
{
private:
    float min_x;
    float max_x;
    float min_y;
    float max_y;

    bool is_initialized;

    FILE *pipe;

    void init(const DataContainer &data);

public:
    Visualization_Gnuplot();
    ~Visualization_Gnuplot();
    void visualize(const Robot &robot, const DataContainer &data);
};
