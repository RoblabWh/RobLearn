#include "simulation2d/lidar.h"

#include <iostream>

#include <eigen3/Eigen/Geometry>

const float EPSILON = 0.00001;

const static int AVX_FLOAT_SIZE = 8;

Lidar::Lidar()
{
    this->hz = 40.0f;
    this->angle_step = 0.00436332312998582394230922692122153178360717972135431364024297860042752278650862360920560392408627370553076123126;
    this->angle_min = -2.3561944902;
    this->angle_max = 2.3561944902;
    this->range_min = 0.06;
    this->range_max = 20;
    this->bias = 0.004;

    create_laser();
}

Lidar::Lidar(float hz, float angle_step, float angle_min, float angle_max, float range_min, float range_max, float bias):hz(hz), angle_step(angle_step), angle_min(angle_min), angle_max(angle_max), range_min(range_min), range_max(range_max), bias(bias)
{
    create_laser();
}

void Lidar::create_laser()
{
    srand(time(NULL)); // random seed for bias
    const int iteration = std::abs((-angle_min + angle_max) / angle_step)+1;

    this->avx_laser_size = calculate_avx_size(iteration);

    this->laser_clear();
    this->laser_reserve(avx_laser_size);


    for (int i = 0; i < iteration; ++i) {
        double angle = static_cast<double>(this->angle_min) + static_cast<double>(this->angle_step) * static_cast<double>(i);
        Eigen::Rotation2Dd rot(angle);
        Eigen::Vector2d vector = rot * Eigen::Vector2d(1,0);

        laser_x.push_back(vector.x());
        laser_y.push_back(vector.y());
        laser_distance.push_back(0);
    }
}

void Lidar::fill_laser_distance_with_range_max()
{
    for (int i = 0; i < laser_distance.size(); ++i) {
        laser_distance[i] = range_max;
    }
}

int Lidar::calculate_avx_size(int size)
{
    return ((size / AVX_FLOAT_SIZE) + ((size % AVX_FLOAT_SIZE == 0)? 0: 1)) * 8;
}

void Lidar::laser_clear()
{
    laser_x.clear();
    laser_y.clear();
    laser_distance.clear();
}

void Lidar::laser_reserve(int size)
{
    laser_x.reserve(size);
    laser_y.reserve(size);
    laser_distance.reserve(size);
}

float Lidar::get_angle_max() const
{
    return this->angle_max;
}

float Lidar::get_angle_min() const
{
    return this->angle_min;
}

float Lidar::get_angle_step() const
{
    return this->angle_step;
}

float Lidar::get_bias() const
{
    return this->bias;
}

float Lidar::get_range_max() const
{
    return this->range_max;
}

float Lidar::get_range_min() const
{
    return this->range_min;
}

float Lidar::get_hz()const
{
    return this->hz;
}

void Lidar::calculate_laser_collision_from_circle(const Eigen::Affine2f &pose, Eigen::Vector2f point, const float radius)
{   
    //Tranform the Circle Point into the coordinatesystem of the lidar
    point = point - pose.translation();
    point = pose.rotation().inverse() * point;

#ifdef USE_AVX
    avx_calculate_laser_collision_from_circle(radius, point.x(), point.y());
#else
    calculate_laser_collision_from_circle(radius, point.x(), point.y());
#endif

    /*** Wolfram alpha version -- obsolete
     * Eigen::Vector2f translation = pose.translation() - point;
     * Eigen::Matrix2f rotation = pose.rotation();
     * const float cos_a = rotation(0,0);
     * const float sin_a = rotation(0,1);
     * this->calculate_laser_collision_from_circle_wolfram(radius, translation.x(), translation.y(), cos_a, sin_a);
     ***/
}

void Lidar::calculate_laser_collision_from_circle(const float radius, const float cx, const float cy)
{
    Eigen::Rotation2Df rot_step(-angle_step);

    Eigen::Vector2f point = Eigen::Rotation2Df(-angle_min) * Eigen::Vector2f(cx,cy);

    const float radius_2 = radius * radius;

    for (int i = 0; i < laser_distance.size(); ++i)
    {
        if (0 < point.x() && std::abs(point.y()) <= radius)
        {
            const float distance = point.x() - std::sqrt(radius_2 - point.y()*point.y());


            if (0 <= distance)
            {
                laser_distance[i] = std::min(laser_distance[i], distance);
            }
        }
        point = rot_step * point;
    }
}

void Lidar::calculate_laser_collision_from_circle_wolfram(const float r, const float x_lidar, const float y_lidar, const float cos_a, const float sin_a)
{
    /***
     * Formular from: http://mathworld.wolfram.com/Circle-LineIntersection.html
     *
     * Intersection between a line and circle-> line ist definite by two points
     * D = (x1*y2 - y2*y1)
     * dx = x2-x1
     * dy = y2-y1
     * dr = sqrt(dx*dx + dy*dy)
     * x_intersect = ( D*dy +- sgn(dy)*dx*sqrt((r*r)*(dr*dr) - (D*D))) / (dr*dr)
     * y_intersect = (-D*dx +- abs(dy)   *sqrt((r*r)*(dr*dr) - (D*D))) / (dr*dr)
     *
     * The circle is in the origin. Lidar scan must transform into circle coordinate. Intersectionpoint are transformed back into the lidar coordinate system.
     ***/
    const float r_2 = r*r;

    for (int i = 0; i < laser_distance.size(); ++i)
    {
        // Tranform the point into circle origin
        const float x1 = x_lidar;
        const float y1 = y_lidar;

        const float x2 = x_lidar + cos_a * this->laser_x[i] - sin_a * this->laser_y[i];
        const float y2 = y_lidar + sin_a * this->laser_x[i] + cos_a * this->laser_y[i];

        const float det = (x1*y2 - x2*y1);
        const float det_2 = det*det;
        const float dx = x2 - x1;
        const float dy = y2 - y1;
        float dr_2 = std::sqrt(dx*dx + dy*dy);
        dr_2 = dr_2*dr_2;

        // Check if a intersection is given
        if (0 <= r_2 * dr_2 - det_2)
        {
            const float sqrt_part = std::sqrt(r_2 * dr_2 - det_2);
            const float x = ((dy < 0)? -1: 1) * dx * sqrt_part;
            const float y = std::abs(dy)           * sqrt_part;

            // Case with transformation in lidar coordinate system
            const float x1_intersect = (det * dy + x) / dr_2 - x_lidar;
            const float x2_intersect = (det * dy - x) / dr_2 - x_lidar;

            const float y1_intersect = (- det * dx + y) / dr_2 - y_lidar;
            const float y2_intersect = (- det * dx - y) / dr_2 - y_lidar;

            // Check if intersectpoints are in the right lidar direction.
            // WARNING:  Acceptance lidar is not in the circle
            if (((0 <= x1_intersect) == (0 <= dx)) && ((0 <= y1_intersect) == (0 <= dy)))
            {
                const float distance1 = std::sqrt(x1_intersect*x1_intersect + y1_intersect*y1_intersect);
                const float distance2 = std::sqrt(x2_intersect*x2_intersect + y2_intersect*y2_intersect);

                laser_distance[i] = std::min(laser_distance[i], std::min(distance1, distance2));
            }
        }
    }
}

void Lidar::apply_bias()
{
    const float div = 1 / static_cast<float>(RAND_MAX);

    for (int i = 0; i < laser_distance.size(); ++i) {
        laser_distance[i] += ((static_cast<float>(rand()) * div) * 2.0f - 1.0f) * this->bias;
    }
}

void Lidar::calculate_laser_collision_from_line(const Eigen::Affine2f &pose, Eigen::Vector2f point1, Eigen::Vector2f point2)
{
    //Tranform the line into the coordinatesystem of the lidar
    point1 -= pose.translation();
    point2 -= pose.translation();

    point1 = pose.rotation().inverse() * point1;
    point2 = pose.rotation().inverse() * point2;

#ifdef USE_AVX
    this->avx_calculate_laser_collision_from_line(point1.x(), point1.y(), point2.x(), point2.y());
#else
    this->calculate_laser_collision_from_line(point1.x(), point1.y(), point2.x(), point2.y());
#endif
}

void Lidar::calculate_laser_collision_from_line(const float x1,const float y1, const float x2, const float y2)
{
    /***
     * Formular from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
     *
     * Intersection of two lines ->Given two points on each line
     * numerator_x = (x1*y2 - y1*x2)(x3 -x4) - (x1 - x2)(x3*y4 - y3*x4)
     * numerator_y = (x1*y2 - y1*x2)(y3 -y4) - (y1 - y2)(x3*y4 - y3*x4)
     * denominator = (x1 - x2)(y3 - y4) - (y1 -y2)(x3 -x4)
     *
     * The formula can be reduced, because the lidar is in the origin -> x4 = y4 = 0
     * numerator_x = (x1*y2 - y1*x2)(x3)
     * numerator_y = (x1*y2 - y1*x2)(y3)
     * denominator = (x1 - x2)(y3) - (y1 -y2)(x3)
     ***/

    // min && max with Epsilon for Bounding Box. case line is horizontal or vertical and the bounding box is a line two.
    const float min_x = std::min(x1,x2) - EPSILON;
    const float min_y = std::min(y1,y2) - EPSILON;
    const float max_x = std::max(x1,x2) + EPSILON;
    const float max_y = std::max(y1,y2) + EPSILON;

    const float x1y2_y1x2 = x1*y2 - y1*x2;
    const float d_x1x2 = x1 - x2;
    const float d_y1y2 = y1 - y2;

    for (int i = 0; i < laser_distance.size(); ++i) {
        const float x3 = laser_x[i];
        const float y3 = laser_y[i];

        const float denominator = d_x1x2 * y3 - d_y1y2 * x3;

        const float x_intersect = x1y2_y1x2 * x3 / denominator;
        const float y_intersect = x1y2_y1x2 * y3 / denominator;

        //check if point is in the right direction and intersect is on line
        if ((((0 <= x_intersect) == (0 <= x3)) && ((0 <= y_intersect) == (0 <= y3)))
                && ((min_x <= x_intersect) && (x_intersect <= max_x))
                && ((min_y <= y_intersect) && (y_intersect <= max_y)) )
        {
            const float distance = std::sqrt(x_intersect*x_intersect + y_intersect * y_intersect);
            laser_distance[i] = std::min(laser_distance[i], distance);
        }
    }
}

void Lidar::print_laser()
{
    for (int i = 0; i < laser_distance.size(); ++i) {
        std::cout << "(" << laser_x[i] << ",\t" << laser_y[i] << "):\t" << laser_distance[i] << std::endl;
    }
}



#ifdef USE_AVX
/****************************************************************
 * AVX PART
 ****************************************************************/
void Lidar::avx_calculate_laser_collision_from_circle(const float radius, const float cx, const float cy)
{
    float array_px[AVX_FLOAT_SIZE];
    float array_py[AVX_FLOAT_SIZE];

    Eigen::Vector2f point = Eigen::Rotation2Df(-angle_min) * Eigen::Vector2f(cx,cy);
    Eigen::Rotation2Df rot_step(-angle_step);

    for (int i = 0; i < AVX_FLOAT_SIZE; ++i)
    {
        array_px[i] = point.x();
        array_py[i] = point.y();
        point = rot_step * point;
    }


    __m256 x = _mm256_loadu_ps(array_px);
    __m256 y = _mm256_loadu_ps(array_py);


    rot_step = Eigen::Rotation2Df(angle_step * static_cast<float>(AVX_FLOAT_SIZE));


    const __m256 cos_a = _mm256_set1_ps(rot_step.matrix()(0,0));
    const __m256 sin_a = _mm256_set1_ps(rot_step.matrix()(0,1));

    const __m256 r_2 = _mm256_set1_ps(radius*radius);

    for (int i = 0; i < avx_laser_size; i += AVX_FLOAT_SIZE)
    {

        // distance = x - sqrt(r*r - y*y);
        __m256 distance = _mm256_sub_ps(x, _mm256_sqrt_ps(_mm256_sub_ps(r_2, _mm256_mul_ps(y,y))));

        // 0 <= distance
        const __m256 mask = _mm256_cmp_ps(_mm256_setzero_ps(),distance, _CMP_LE_OQ);

        __m256 distance_load = avx_load_laser_distance(i);

        distance = _mm256_min_ps(distance, distance_load);

        distance = _mm256_blendv_ps(distance_load, distance, mask);

        avx_store_laser_distance(i, distance);

        //point = rot_step * point;
        const __m256 tmp_x = x;

        // x = cos_a * x - sin_a * y
        x = _mm256_sub_ps(_mm256_mul_ps(cos_a, x), _mm256_mul_ps(sin_a, y));

        // y = sin_a * x + cos_a * y
        y = _mm256_add_ps(_mm256_mul_ps(sin_a, tmp_x), _mm256_mul_ps(cos_a, y));




    }
}

void Lidar::avx_calculate_laser_collision_from_line(const float px1, const float py1, const float px2, const float py2)
{
    /***
     * Formular from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
     *
     * Intersection of two lines ->Given two points on each line
     * numerator_x = (x1*y2 - y1*x2)(x3 -x4) - (x1 - x2)(x3*y4 - y3*x4)
     * numerator_y = (x1*y2 - y1*x2)(y3 -y4) - (y1 - y2)(x3*y4 - y3*x4)
     * denominator = (x1 - x2)(y3 - y4) - (y1 -y2)(x3 -x4)
     *
     * The formula can be reduced, because the lidar is in the origin -> x4 = y4 = 0
     * numerator_x = (x1*y2 - y1*x2)(x3)
     * numerator_y = (x1*y2 - y1*x2)(y3)
     * denominator = (x1 - x2)(y3) - (y1 -y2)(x3)
     ***/

    // min && max with Epsilon for Bounding Box. case line is horizontal or vertical and the bounding box is a line two.
    const __m256 min_x = _mm256_set1_ps(std::min(px1,px2) - EPSILON);
    const __m256 min_y = _mm256_set1_ps(std::min(py1,py2) - EPSILON);
    const __m256 max_x = _mm256_set1_ps(std::max(px1,px2) + EPSILON);
    const __m256 max_y = _mm256_set1_ps(std::max(py1,py2) + EPSILON);

    const __m256 x1 = _mm256_set1_ps(px1);
    const __m256 y1 = _mm256_set1_ps(py1);

    const __m256 x2 = _mm256_set1_ps(px2);
    const __m256 y2 = _mm256_set1_ps(py2);

    const __m256 x1y2_y1x2 = _mm256_sub_ps(_mm256_mul_ps(x1,y2), _mm256_mul_ps(y1,x2)); // x1*y2 - y1*x2
    const __m256 d_x1x2 = _mm256_sub_ps(x1, x2); // x1 - x2
    const __m256 d_y1y2 = _mm256_sub_ps(y1, y2); // y1 - y2

    for (int i = 0; i < this->avx_laser_size; i += AVX_FLOAT_SIZE) {
        const __m256 x3 = this->avx_load_laser_x(i);
        const __m256 y3 = this->avx_load_laser_y(i);

        // denominator =  d_x1x2 * y3 - d_y1y2 * x3;
        const __m256 denominator = _mm256_sub_ps(_mm256_mul_ps(d_x1x2, y3), _mm256_mul_ps(d_y1y2, x3));

        // x_intersect = x1y2_y1x2 * x3 / denominator
        const __m256 x_intersect = _mm256_div_ps(_mm256_mul_ps(x1y2_y1x2, x3), denominator);
        // y_intersect = x1y2_y1x2 * y3 / denominator
        const __m256 y_intersect = _mm256_div_ps(_mm256_mul_ps(x1y2_y1x2, y3), denominator);



        // ((0 <= x_intersect) == (0 <= x3)) && ((0 <= y_intersect) == (0 <= y3))
        __m256 mask = _mm256_and_ps(
                            avx_nxor(
                                    _mm256_cmp_ps(_mm256_setzero_ps(),x_intersect, _CMP_LE_OQ),
                                    _mm256_cmp_ps(_mm256_setzero_ps(),x3, _CMP_LE_OQ)),
                            avx_nxor(
                                    _mm256_cmp_ps(_mm256_setzero_ps(),y_intersect, _CMP_LE_OQ),
                                    _mm256_cmp_ps(_mm256_setzero_ps(),y3, _CMP_LE_OQ)));

        // mask && ((min_x <= x_intersect) && (x_intersect <= max_x))
        mask = _mm256_and_ps(mask,
                    _mm256_and_ps(
                        _mm256_cmp_ps(min_x,x_intersect, _CMP_LE_OQ),
                        _mm256_cmp_ps(x_intersect,max_x, _CMP_LE_OQ)));

        // mask && ((min_y <= y_intersect) && (y_intersect <= max_y)) )
        mask = _mm256_and_ps(mask,
                    _mm256_and_ps(
                        _mm256_cmp_ps(min_y,y_intersect, _CMP_LE_OQ),
                        _mm256_cmp_ps(y_intersect,max_y, _CMP_LE_OQ)));


        __m256 distance = _mm256_sqrt_ps(_mm256_add_ps(_mm256_mul_ps(x_intersect, x_intersect), _mm256_mul_ps(y_intersect, y_intersect)));

        __m256 distance_load = avx_load_laser_distance(i);
        distance = _mm256_min_ps(distance, distance_load);

        distance = _mm256_blendv_ps(distance_load, distance, mask);

        avx_store_laser_distance(i, distance);

    }
}


#endif
