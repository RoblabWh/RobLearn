#include "simulation2d/simulation2d.h"

#include <chrono>
#include <fstream>
#include <boost/tokenizer.hpp>



Simulation2D::Simulation2D()
{
    collision = false;
}

/**
 * Old function which simulate every laserscan.
 *
void Simulation2D::step(const float linear_velocity, const float angular_velocity, int skip_number)
{
    if (skip_number < 1)
    {
        skip_number = 1;
    }
    float time_step_end = this->time_step_check_collision * skip_number;
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
*/

void Simulation2D::step(const float linear_velocity, const float angular_velocity, int skip_number)
{
    if (skip_number < 1)
    {
        skip_number = 1;
    }

    float time_step_end = this->time_step_check_laser * skip_number;
    float time_step = 0.0f;

    while (time_step < time_step_end && !collision)
    {
        robot.move(linear_velocity * time_step_check_collision, angular_velocity * time_step_check_collision);
        collision = data.calculate_robot_collision(robot);

        time_step += time_step_check_collision;
    }

    if(!collision)
    {
        float time_step_to_laserscan = time_step - time_step_end;
        robot.move(linear_velocity * time_step_to_laserscan, angular_velocity * time_step_to_laserscan);
    }


    data.calculate_lidar_collision(robot);
}

void Simulation2D::calculate_iteration()
{
    time_step_check_collision = robot.get_collision_radius() / robot.get_max_velocity();
    time_step_check_laser= 1 / robot.get_lidar().get_hz();
}

bool Simulation2D::init(std::string world)
{
    bool success = true;
    collision = false;
    calculate_iteration();

    if (world.compare("") == 0)
    {
        load_default_world();
    }
    else
    {
        success = load_world(world);
    }

    return success;
}


Eigen::Affine2f Simulation2D::get_robot_pose()
{
    return robot.get_pose();
}


void Simulation2D::set_robot_pose(const float x, const float y, const float orientation)
{
    robot.set_pose(x,y,orientation);
    collision = false;
}

void Simulation2D::visualize(const float end_x, const float end_y, const float end_radius)
{
    visualization.visualize(robot, data, Circle(end_x, end_y, end_radius));
}

void Simulation2D::load_default_world()
{
    std::vector<Line> lines;
    std::vector<Circle> circles;
    // walls
    lines.push_back(Line(10,10,10,-10));
    lines.push_back(Line(10,-10,-10,-10));
    lines.push_back(Line(-10,-10,-10,10));
    lines.push_back(Line(-10,10,8,10));

    // Room1
    lines.push_back(Line(-8,-10,-8,-7));
    lines.push_back(Line(-6,-7,-5,-10));
    lines.push_back(Line(-10,-5,-7,-4));
    lines.push_back(Line(-3, -10,-1,-8));
    lines.push_back(Line(-7,-4,-5,-5));
    lines.push_back(Line(-5,-5,-4.25,-7.5));

    circles.push_back(Circle(-7, -7, 1));
    circles.push_back(Circle(-3.75,-7.5,0.5));
    circles.push_back(Circle(-4,-3,0.5));


    //Hall 1-2
    lines.push_back(Line(-5,-5, -7, 0));
    lines.push_back(Line(-5,-5,-3.3, -7.25));
    lines.push_back(Line(-7, 0, -10, 0));
    lines.push_back(Line(-2,-2, -5, 1));

    //Room 2
    lines.push_back(Line(-5, 1, -4, 3));
    lines.push_back(Line(-4, 3, -6, 5));
    lines.push_back(Line(-6, 5, -4, 6));
    lines.push_back(Line(-4, 6,-3,7));
    lines.push_back(Line(-4, 10,-3,9));

    circles.push_back(Circle(-6.75,2.75,1));
    circles.push_back(Circle(-5.75,7.25,1));

    // Hall 2-3
    lines.push_back(Line(-3,7, -2,4));
    lines.push_back(Line(-2,4, 0, 0));
    lines.push_back(Line(0, 0,-2, -2));
    lines.push_back(Line(-2,-2,-2,-6));
    lines.push_back(Line(-2,-6,0,-10));
    lines.push_back(Line(-3,9, -1,8));
    lines.push_back(Line(-1,8,2,4));
    lines.push_back(Line(2,4,3,-2));

    circles.push_back(Circle(0,1,0.3));
    circles.push_back(Circle(1,4,0.4));
    circles.push_back(Circle(-2,6,0.2));

    // Room 3
    lines.push_back(Line(3,-2, 6, -2));
    lines.push_back(Line(8, -2,10,-4));

    lines.push_back(Line(1,-3, 1, -5));
    lines.push_back(Line(1,-5, 0, -5));
    lines.push_back(Line(0, -5,1,-3));

    lines.push_back(Line(7, -5, 6.5,-6));
    lines.push_back(Line(6.5,-6, 5,-6.5));
    lines.push_back(Line(5,-6.5, 7,-6.25));
    lines.push_back(Line(7,-6.25, 7, -5));

    circles.push_back(Circle(4,-4,1.75));
    circles.push_back(Circle(2,-7,0.8));
    circles.push_back(Circle(5,-9.2,0.7));
    circles.push_back(Circle(8.5,-7,0.2));
    circles.push_back(Circle(6.5,-3.5,0.3));

    // Hall final
    lines.push_back(Line(8,-2, 10,2));
    lines.push_back(Line(6,-2, 8,2));
    lines.push_back(Line(8,2,3,2));
    lines.push_back(Line(10,4, 5,4));
    lines.push_back(Line(3,2,0,8));
    lines.push_back(Line(0,8,2,10));
    lines.push_back(Line(5,4,4,6));
    lines.push_back(Line(4,6,5,7));
    lines.push_back(Line(5,7,6,5));
    lines.push_back(Line(6,5,10,5));

    lines.push_back(Line(6,10,7,7));
    lines.push_back(Line(7,7,8,7));
    lines.push_back(Line(8,7,8,10));

    circles.push_back(Circle(3,7.75,0.8));
    circles.push_back(Circle(4,5,0.2));
    circles.push_back(Circle(7,3.6,0.15));
    circles.push_back(Circle(3.7,2.7,0.3));

    data.set_world(lines, circles);
}

bool Simulation2D::load_world(const std::string &world)
{
    std::cout << "-> Load world: " << world << std::endl;

    std::vector<Line> lines;
    std::vector<Circle> circles;

    std::ifstream file;

    file.open(world);

    if (!file.is_open()) {
        std::cerr << "Couldn not read file: " << world << std::endl;
        return false;
    }

    std::string line;

    int line_counter = 0;

    while(std::getline(file, line))
    {
        line_counter++;

        std::vector<std::string> line_split;

        for (int i = 0; i < line.size(); ++i) {
            int begin = i;

            while(line[i] != ' ' && i < line.size()) i++;

            std::string substring(line.substr(begin, i - begin));

            if (!substring.empty()) {
                line_split.push_back(substring);
            }
        }

        // continue if line is empty
        if(line_split.empty())
            continue;

        // continue if line starts with # -> ignore comment line
        if (line_split[0][0] == '#')
            continue;

        // parse circle with 3 index
        if (line_split.size() == 3) {
            try {
                float x = std::stof(line_split[0]);
                float y = std::stof(line_split[1]);
                float r = std::stof(line_split[2]);

                circles.push_back(Circle(x,y,r));
            } catch (...) {
                std::cerr << "   ERROR: parsing float values for circle! [Line: " << line_counter << "]" << std::endl;
            }
        }
        // parse line with 4 index
        else if (line_split.size() == 4) {
            try {
                float x1 = std::stof(line_split[0]);
                float y1 = std::stof(line_split[1]);
                float x2 = std::stof(line_split[2]);
                float y2 = std::stof(line_split[3]);
                lines.push_back(Line(x1,y1,x2,y2));
            } catch (...) {
                std::cerr << "   ERROR: parsing float values for line! [Line: " << line_counter << "]" << std::endl;
            }
        }
        // print error and go on
        else
        {
            std::cerr << "   WARN: unknwon split size of " << line_split.size() << "! [Line: " << line_counter << "]";
        }
    }

    data.set_world(lines, circles);

    return true;
}
