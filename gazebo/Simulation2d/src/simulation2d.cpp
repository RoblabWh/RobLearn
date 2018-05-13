#include "simulation2d/simulation2d.h"

#include <chrono>



Simulation2D::Simulation2D()
{

}

void Simulation2D::
step(const float linear_velocity, const float angular_velocity)
{
    float time_step = 0.0f;
    float time_step_check_laser_next = time_step_check_laser;

    while (time_step < time_step_end && !collision)
    {
        time_step += time_step_check_collision;
        if (time_step < time_step_check_laser_next) {
            robot.move(linear_velocity * time_step_check_collision, angular_velocity * time_step_check_collision);
            collision = data.calculate_robot_collision(robot);
        }
        else
        {
            const float time_step_diff = time_step - time_step_check_laser_next;
            const float time_step_to_simulate = time_step_check_collision - time_step_diff;
            robot.move(linear_velocity * time_step_to_simulate, angular_velocity * time_step_to_simulate);

            collision = data.calculate_robot_collision(robot);

            if (!collision) {
                data.calculate_lidar_collision(robot);
            }

            time_step_check_laser_next += time_step_check_laser;
            time_step -= time_step_diff;
        }

    }
}

void Simulation2D::calculate_iteration()
{
    skip_laser = (0 < skip_laser)? skip_laser: 1;
    time_step_check_collision = robot.get_collision_radius() / robot.get_max_velocity();
    time_step_check_laser= 1 / robot.get_lidar().get_hz();
    time_step_end = skip_laser *  time_step_check_laser;
}

bool Simulation2D::init()
{
    collision = false;
    calculate_iteration();
    visualization.init(data);

}

Eigen::Affine2f Simulation2D::get_robot_pose()
{
    return robot.get_pose();
}

void Simulation2D::set_robot_pose(const float x, const float y, const float orientation)
{
    robot.set_pose(x,y,orientation);
}

void Simulation2D::visualize()
{
    visualization.visualize(robot, data);
}

void Simulation2D::test()
{
    std::vector<Line> lines;

    lines.push_back(Line(10,10,10,-10));
    lines.push_back(Line(10,-10,-10,-10));
    lines.push_back(Line(-10,-10,-10,10));
    lines.push_back(Line(-10,10,10,10));

    /**
    lines.push_back(Line(0,1,1,0));
    lines.push_back(Line(1,0,0,-1));
    lines.push_back(Line(0,-1,-1,0));
    lines.push_back(Line(-1,0,0,1));
    */

    data.set_lines(lines);

    std::vector<Circle> circles;
    //circles.push_back(Circle(3,0,2));
    circles.push_back(Circle(-6,6,3));
    circles.push_back(Circle(7,-7,2));
    circles.push_back(Circle(6,0,1));
    data.set_circles(circles);

    robot.set_position(0,0);
   // robot.set_orientation(0.7853);

    data.calculate_robot_collision(robot);
}


