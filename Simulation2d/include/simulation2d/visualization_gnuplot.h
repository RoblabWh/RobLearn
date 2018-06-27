#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"

#include <eigen3/Eigen/Core>

class Visualization_Gnuplot
{
private:
    bool is_initialized;

    std::string world;

    FILE *pipe;

    void init(const DataContainer &data);
    void construct_world(const DataContainer &data);

public:
    Visualization_Gnuplot();
    ~Visualization_Gnuplot();
    void visualize(const Robot &robot, const DataContainer &data);
};
