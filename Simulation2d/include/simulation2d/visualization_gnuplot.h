#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"

#include <eigen3/Eigen/Core>

class Visualization_Gnuplot
{
private:
    /**
     * @brief is_initialized Flag when the visualzation pipeline is initialzied.
     */
    bool is_initialized;

    /**
     * @brief world Gnuplot string for the static lines and circles.
     */
    std::string world;

    /**
     * @brief pipe Pipeline for the gnuplot.
     */
    FILE *pipe;

    /**
     * @brief init Initializied the gnuplot pipeline.
     * @param data Datacontainer.
     */
    void init(const DataContainer &data);
    /**
     * @brief construct_world Prepare the world string.
     * @param data Datacontainer.
     */
    void construct_world(const DataContainer &data);

public:
    /**
     * @brief Visualization_Gnuplot Constructor for the Visualization_Gnuplot.
     */
    Visualization_Gnuplot();
    ~Visualization_Gnuplot();
    /**
     * @brief visualize Viszualize the the current scene with gnuplot.
     * @param robot Robot,
     * @param data Datacontainer.
     * @param end_node Target node.
     */
    void visualize(const Robot &robot, const DataContainer &data, const Circle &end_node);
};
