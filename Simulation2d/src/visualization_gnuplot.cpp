#include "simulation2d/visualization_gnuplot.h"
#include <stdio.h>

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
    pipe = popen("/usr/bin/gnuplot -geometry 800x800 -persist", "w");

    for (int i = 0; i < data.get_line_size(); ++i) {
        this->min_x = std::min(min_x, std::min(data.get_line_x1_at(i), data.get_line_x2_at(i)));
        this->min_y = std::min(min_y, std::min(data.get_line_y1_at(i), data.get_line_y2_at(i)));

        this->max_x = std::max(max_x, std::max(data.get_line_x1_at(i), data.get_line_x2_at(i)));
        this->max_y = std::max(max_y, std::max(data.get_line_y1_at(i), data.get_line_y2_at(i)));
    }

    for (int i = 0; i < data.get_circle_size(); ++i) {
        this->min_x = std::min(min_x, data.get_circle_x_at(i));
        this->min_y = std::min(min_y, data.get_circle_y_at(i));

        this->max_x = std::max(max_x, data.get_circle_x_at(i));
        this->max_y = std::max(max_x, data.get_circle_y_at(i));
    }

    is_initialized = true;
}

void Visualization_Gnuplot::visualize(const Robot &robot, const DataContainer &data)
{
    if (!is_initialized) {
        init(data);
    }

    if (!pipe) {
        std::cout << "could create gnuplotPipe:" <<std:: endl;
        return;
    }

    fprintf(pipe, "set xrange [%f:%f]\n",(min_x - 1),(max_x + 1));
    fprintf(pipe, "set yrange [%f:%f]\n",(min_y - 1),(max_y + 1));


    fprintf(pipe, "plot ");
    if (data.get_line_size() != 0)
    {
        fprintf(pipe, "'-' using 1:2:3:4 with vectors nohead lc 'blue', ");
    }
    if (data.get_circle_size() != 0)
    {
        fprintf(pipe, "'-' using 1:2:3 with circles lc 'blue', ");
    }
    fprintf(pipe, "'-' using 1:2:3:4 with vectors nohead lc 'green', '-' using 1:2:3 with circles lc 'red', '-' using 1:2:3:4 with vectors lc 'red'\n");



    //print lines
    for (int i = 0; i < data.get_line_size(); ++i) {
        fprintf(pipe, "%f %f %f %f\n", data.get_line_x1_at(i), data.get_line_y1_at(i), (data.get_line_x2_at(i) - data.get_line_x1_at(i)), (data.get_line_y2_at(i) - data.get_line_y1_at(i)));
    }
    fprintf(pipe, "EOF\n");

    //print circles
    for (int i = 0; i < data.get_circle_size(); ++i) {
        fprintf(pipe, "%f %f %f\n", data.get_circle_x_at(i), data.get_circle_y_at(i), data.get_circle_radius_at(i));
    }
    fprintf(pipe, "EOF\n");





    //print laser
    Eigen::Affine2f pose = robot.get_pose();
    for (int i = 0; i < robot.get_lidar().get_laser_size(); ++i) {
        Eigen::Vector2f laser = pose.rotation() * robot.get_lidar().get_laser_point_distance_at(i);
        fprintf(pipe, "%f %f %f %f\n", robot.get_position_x(), robot.get_position_y(), laser.x(), laser.y());
    }
    fprintf(pipe, "EOF\n");

    //print robot
    fprintf(pipe, "%f %f %f\n", robot.get_position_x(), robot.get_position_y(), robot.get_collision_radius());

    fprintf(pipe, "EOF\n");


    //print robot direction
    Eigen::Vector2f direction = pose.rotation() * Eigen::Vector2f(robot.get_collision_radius(),0);
    fprintf(pipe, "%f %f %f %f\n", robot.get_position_x(), robot.get_position_y(), direction.x(), direction.y());

    fprintf(pipe, "EOF\n");


    fflush(pipe);


    /**

    for (int i = 0; i < KOEFFIZIENT_SIZE; ++i) {
        fprintf(gnuplotPipe,"%f*x**%i+",population[0].koeffizient[i],i);
    }

    fprintf(gnuplotPipe,"0, '-' using 1:2 w p, '-' using 1:2 w l\n");



    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        fprintf(gnuplotPipe, "%f %f \n", samples[i].x, samples[i].y_noise);
    }
    fprintf(gnuplotPipe, "EOF\n");

    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        fprintf(gnuplotPipe, "%f %f \n", samples[i].x, samples[i].y);
    }

    fprintf(gnuplotPipe, "EOF\n");


    fflush(gnuplotPipe);


    usleep(1000000.0f/PLOT_SPEED_HZ);
    */
}
