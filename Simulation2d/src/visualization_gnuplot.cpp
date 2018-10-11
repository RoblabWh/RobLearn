#include "simulation2d/visualization_gnuplot.h"
#include <stdio.h>
#include <limits>

Visualization_Gnuplot::Visualization_Gnuplot()
{
    is_initialized = false;
}

Visualization_Gnuplot::~Visualization_Gnuplot()
{
    if (is_initialized) {
        pclose(pipe);
    }
}

void Visualization_Gnuplot::init(const DataContainer &data)
{
    construct_world(data);

    pipe = popen("/usr/bin/gnuplot -geometry 800x800 -persist", "w");

    is_initialized = true;
}

void Visualization_Gnuplot::construct_world(const DataContainer &data)
{
    std::stringstream ss;
    ss << "set xrange [" << (data.get_area_min_x() - 1) << ":" << (data.get_area_max_x() + 1) << "]\n";
    ss << "set yrange [" << (data.get_area_min_y() - 1) << ":" << (data.get_area_max_y() + 1) << "]\n";
    ss << "plot ";

    if (data.get_line_size() != 0)
    {
        ss << "'-' using 1:2:3:4 with vectors nohead lc 'blue', ";
    }
    if (data.get_circle_size() != 0)
    {
        ss << "'-' using 1:2:3 with circles lc 'blue', ";
    }

    ss << "'-' using 1:2:3:4 with vectors nohead lc 'green', '-' using 1:2:3 with circles lc 'magenta' fill solid border, '-' using 1:2:3 with circles lc 'red', '-' using 1:2:3:4 with vectors lc 'red'\n";

    // create lines
    if (data.get_line_size() != 0) {
        for (int i = 0; i < data.get_line_size(); ++i) {
            ss << data.get_line_x1_at(i) << " " << data.get_line_y1_at(i) << " " << (data.get_line_x2_at(i) - data.get_line_x1_at(i)) << " " << (data.get_line_y2_at(i) - data.get_line_y1_at(i)) << "\n";
        }
        ss << "EOF\n";
    }

    // create circles
    if (data.get_circle_size() != 0)
    {
        for (int i = 0; i < data.get_circle_size(); ++i) {
            ss << data.get_circle_x_at(i) << " " << data.get_circle_y_at(i) << " " << data.get_circle_radius_at(i) << "\n";
        }
        ss << "EOF\n";

    }

    world = ss.str();

}

void Visualization_Gnuplot::visualize(const Robot &robot, const DataContainer &data, const Circle &end_node)
{
    // init the pipeline if not initializied
    if (!is_initialized) {
        init(data);
    }

    if (!pipe) {
        std::cout << "could create gnuplotPipe:" <<std:: endl;
        return;
    }

    std::stringstream ss;

    // create laser
    Eigen::Affine2f pose = robot.get_pose();
    for (int i = 0; i < robot.get_lidar().get_laser_size(); ++i) {
        Eigen::Vector2f laser = pose.rotation() * robot.get_lidar().get_laser_point_distance_at(i);
        ss << robot.get_position_x() << " " << robot.get_position_y() << " " << laser.x() << " " << laser.y() << "\n";
    }
    ss << "EOF\n";

    // create end node
    ss << end_node.get_x() << " " << end_node.get_y() << " " << end_node.get_radius() << "\n";

    ss << "EOF\n";

    // create robot
    ss << robot.get_position_x() << " " << robot.get_position_y() << " " << robot.get_collision_radius() << "\n";

    ss << "EOF\n";


    // create robot direction
    Eigen::Vector2f direction = pose.rotation() * Eigen::Vector2f(robot.get_collision_radius(),0);
    ss << robot.get_position_x() << " " << robot.get_position_y() << " " << direction.x() << " " << direction.y() << "\n";

    ss << "EOF\n";

    fprintf(pipe, "%s%s", world.c_str(), ss.str().c_str());

    fflush(pipe);
}
