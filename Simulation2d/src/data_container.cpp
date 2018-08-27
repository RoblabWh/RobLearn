#include "simulation2d/data_container.h"

#include <iostream>

const static int AVX_FLOAT_SIZE = 8;

DataContainer::DataContainer()
{
    this->avx_line_size = 0;
    this->avx_circle_size = 0;
}

void DataContainer::set_world(const std::vector<Line> &lines, const std::vector<Circle> &circles)
{
    this->set_lines(lines);
    this->set_circles(circles);
    this->set_area();
}

void DataContainer::set_lines(const std::vector<Line> &lines)
{
    this->avx_line_size = calculate_avx_size(lines.size());

    this->line_clear();
    this->line_reserve(avx_line_size);

    for (int i = 0; i < lines.size(); ++i)
    {
        line_x1.push_back(lines[i].get_point1_x());
        line_y1.push_back(lines[i].get_point1_y());
        line_x2.push_back(lines[i].get_point2_x());
        line_y2.push_back(lines[i].get_point2_y());
        line_distance.push_back(0);
    }
}

void DataContainer::set_area()
{
    this->area_min_x = std::numeric_limits<float>::max();
    this->area_min_y = std::numeric_limits<float>::max();
    this->area_max_x = -std::numeric_limits<float>::max();
    this->area_max_y = -std::numeric_limits<float>::max();

    for (int i = 0; i < line_distance.size(); ++i) {
        this->area_min_x = std::min(area_min_x, std::min(line_x1[i], line_x2[i]));
        this->area_min_y = std::min(area_min_y, std::min(line_y1[i], line_y2[i]));

        this->area_max_x = std::max(area_max_x, std::max(line_x1[i], line_x2[i]));
        this->area_max_y = std::max(area_max_y, std::max(line_y1[i], line_y2[i]));
    }

    for (int i = 0; i < circle_distance.size(); ++i) {
        this->area_min_x = std::min(area_min_x, circle_x[i]);
        this->area_min_y = std::min(area_min_y, circle_y[i]);

        this->area_max_x = std::max(area_max_x, circle_x[i]);
        this->area_max_y = std::max(area_max_y, circle_y[i]);
    }
}


void DataContainer::set_circles(const std::vector<Circle> &circles)
{
    this->avx_circle_size = calculate_avx_size(circles.size());

    this->circle_clear();
    this->circle_reserve(avx_circle_size);

    for (int i = 0; i < circles.size(); ++i)
    {
        circle_x.push_back(circles[i].get_x());
        circle_y.push_back(circles[i].get_y());
        circle_radius.push_back(circles[i].get_radius());
        circle_distance.push_back(0);
    }
}

int DataContainer::calculate_avx_size(int size)
{
    return ((size / AVX_FLOAT_SIZE) + ((size % AVX_FLOAT_SIZE == 0)? 0: 1)) * 8;
}

void DataContainer::line_clear()
{
    line_x1.clear();
    line_y1.clear();
    line_x2.clear();
    line_y2.clear();
    line_distance.clear();
}

void DataContainer::line_reserve(int size)
{
    line_x1.reserve(size);
    line_y1.reserve(size);
    line_x2.reserve(size);
    line_y2.reserve(size);
    line_distance.reserve(size);
}

void DataContainer::circle_clear()
{
    circle_x.clear();
    circle_y.clear();
    circle_radius.clear();
    circle_distance.clear();
}

void DataContainer::circle_reserve(int size)
{
    circle_x.reserve(size);
    circle_y.reserve(size);
    circle_radius.reserve(size);
    circle_distance.reserve(size);
}


void DataContainer::calculate_line_distance_from_point(const float x, const float y)
{
    const float x0 = x;
    const float y0 = y;

    for (int i = 0; i < line_distance.size(); ++i) {
        const float x1 = line_x1[i];
        const float y1 = line_y1[i];

        const float x2 = line_x2[i];
        const float y2 = line_y2[i];

        float numerator = std::abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1);

        float dx = x2 - x1;
        float dy = y2 - y1;
        float denominator = std::sqrt(dx * dx + dy * dy);

        line_distance[i] = numerator / denominator;
    }
}



bool DataContainer::calculate_robot_collision(Robot &robot)
{
#ifdef USE_AVX
    this->avx_calculate_circle_distance_from_point(robot.get_position_x(), robot.get_position_y());
    this->avx_calculate_line_distance_from_point(robot.get_position_x(), robot.get_position_y());
#else
    this->calculate_circle_distance_from_point(robot.get_position_x(), robot.get_position_y());
    this->calculate_line_distance_from_point(robot.get_position_x(), robot.get_position_y());
#endif


    // Check the circle collision with the robot
    for (int i = 0; i < circle_distance.size(); ++i) {
        if (circle_distance[i] <= (robot.get_collision_radius() + circle_radius[i])) {
            return true;
        }
    }

    // Check the line collsion with the robot
    for (int i = 0; i < line_distance.size(); ++i) {
        if (line_distance[i] <= robot.get_collision_radius() && is_point_in_line_segment(robot.get_position_x(), robot.get_position_y(), i))
        {
            return true;
        }
    }

    // check if robot is out of area
    if (robot.get_position_x() <= this->area_min_x || this->area_max_x <= robot.get_position_x() || robot.get_position_y() <= this->area_min_y || this->area_max_y <= robot.get_position_y()) {
        return true;
    }

    return false;
}



void DataContainer::calculate_circle_distance_from_point(const float x, const float y)
{
    const float x0 = x;
    const float y0 = y;

    for (int i = 0; i < circle_distance.size(); ++i) {
        const float dx = circle_x[i] - x0;
        const float dy = circle_y[i] - y0;

        circle_distance[i] = std::sqrt(dx*dx + dy*dy);
    }
}

void DataContainer::calculate_lidar_collision(Robot &robot)
{
    Eigen::Affine2f pose = robot.get_pose();
    Lidar &lidar = robot.get_lidar();

    // fill lidar with max range
    lidar.fill_laser_distance_with_range_max();


    // lidar collision with the lines from the world
    for (int i = 0; i < line_distance.size(); ++i) {
        if (line_distance[i] <= lidar.get_range_max())
        {
            lidar.calculate_laser_collision_from_line(pose, this->get_line_point1_at(i), this->get_line_point2_at(i));
        }
    }

    // lidar collision with the circles from the world
    for (int i = 0; i < circle_distance.size(); ++i)
    {
        if (circle_distance[i] <= lidar.get_range_max())
        {
            lidar.calculate_laser_collision_from_circle(pose, this->get_circle_point_at(i), this->get_circle_radius_at(i));
        }  
    }

    // make the lidar noisy
    lidar.apply_bias();
}


/**************************************************************************************
 * AVX PART
 **************************************************************************************/
#ifdef USE_AVX
void DataContainer::avx_calculate_line_distance_from_point(const float x,const  float y)
{
    /***
     * Formular from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
     * numerators = |(y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1|
     * denominators =  sqrt(pow(y2-y1) + pow(x2-x1))
     ***/
    __m256 x0 = _mm256_set1_ps(x);
    __m256 y0 = _mm256_set1_ps(y);

    for (int i = 0; i < avx_line_size; i += AVX_FLOAT_SIZE)
    {

        __m256 x1 = avx_load_line_x1(i);
        __m256 y1 = avx_load_line_y1(i);

        __m256 x2 = avx_load_line_x2(i);
        __m256 y2 = avx_load_line_y2(i);

        // (y2-y1)*x0
        __m256 numerator = _mm256_mul_ps(_mm256_sub_ps(y2,y1),x0);

        // numerator - (x2-x1)*y0
        numerator = _mm256_sub_ps(numerator, _mm256_mul_ps(_mm256_sub_ps(x2,x1), y0));

        // numerator + x2*y1
        numerator = _mm256_add_ps(numerator, _mm256_mul_ps(x2,y1));

        // numerator - y2*x1
        numerator = _mm256_sub_ps(numerator, _mm256_mul_ps(y2,x1));

        // |numerator|
        numerator = avx_abs(numerator);

        // pow(y2-y1)
        __m256 tmp = _mm256_sub_ps(y2,y1);
        __m256 denominator = _mm256_mul_ps(tmp, tmp);

        // pow(x2-x1)
        tmp = _mm256_sub_ps(x2,x1);
        tmp = _mm256_mul_ps(tmp, tmp);

        // denominator + tmp
        denominator = _mm256_add_ps(denominator, tmp);

        // sqrt(denominator)
        denominator = _mm256_sqrt_ps(denominator);

        avx_store_line_distance(i, _mm256_div_ps(numerator, denominator));
    }
}

void DataContainer::avx_calculate_circle_distance_from_point(const float x, const float y)
{
    __m256 x0 = _mm256_set1_ps(x);
    __m256 y0 = _mm256_set1_ps(y);

    for (int i = 0; i < avx_circle_size; i += AVX_FLOAT_SIZE)
    {
        __m256 x1 = avx_load_circle_x(i);
        __m256 y1 = avx_load_circle_y(i);

        x1 = _mm256_sub_ps(x1, x0);
        y1 = _mm256_sub_ps(y1, y0);

        x1 = _mm256_mul_ps(x1, x1);
        y1 = _mm256_mul_ps(y1, y1);

        x1 = _mm256_sqrt_ps(_mm256_add_ps(x1,y1));

        avx_store_circle_distance(i,x1);
    }
}

#endif
