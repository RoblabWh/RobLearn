#pragma once

#include <vector>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class Lidar
{
private:
    /**
     * @brief avx_laser_size The size of the vector for the avx.
     */
    int avx_laser_size;

    /**
     * @brief laser_x X value of the unit vector for the lidar orientation.
     */
    std::vector<float> laser_x;
    /**
     * @brief laser_y Y value of the unit vector for the lidar orientation.
     */
    std::vector<float> laser_y;
    /**
     * @brief laser_distance Vector of the calculated distance for each lidar.
     */
    std::vector<float> laser_distance;

    /**
     * @brief hz Rate of the lidar in hertz.
     */
    float hz;
    /**
     * @brief angle_min Minimun angle of the lidar.
     */
    float angle_min;
    /**
     * @brief angle_max Maximum angle of the lidar.
     */
    float angle_max;
    /**
     * @brief angle_step Steps beetween the lidar.
     */
    float angle_step;
    /**
     * @brief range_min Minimum range of the lidar.
     */
    float range_min;
    /**
     * @brief range_max Maximum range of the lidar.
     */
    float range_max;
    /**
     * @brief bias Bias factor of each lidar.
     */
    float bias;


    /**
     * @brief laser_clear Clear all vector.
     */
    void laser_clear();
    /**
     * @brief laser_reserve Reserve the size of all vectors
     * @param size Reserve size.
     */
    void laser_reserve(int size);

    /**
     * @brief create_laser Create the lidar. Fill the vector with unit vector for the orientation.
     */
    void create_laser();
    /**
     * @brief calculate_avx_size Calculate the avx size for the vector.
     * @param size Normal size.
     * @return Avx vector size.
     */
    int calculate_avx_size(int size);


    /**
     * @brief calculate_laser_collision_from_line Calculate the lidar collision from a line.
     * @param x1 X posiiton of the first point.
     * @param y1 Y posiiton of the first point.
     * @param x2 X posiiton of the second point.
     * @param y2 Y posiiton of the second point.
     */
    void calculate_laser_collision_from_line(const float x1, const float y1, const float x2, const float y2);

    /**
     * @brief calculate_laser_collision_from_circle_wolfram Calculate the lidar collision from a circle. Use the formula from wolfram alpha.
     * @param r Radius of the Circle.
     * @param x_lidar X position of the lidar.
     * @param y_lidar Y Position of the lidar.
     * @param cos_a Cosinus orientation of the robot,
     * @param sin_a Sinus orientation of the robot.
     */
    void calculate_laser_collision_from_circle_wolfram(const float r, const float x_lidar, const float y_lidar, const float cos_a, const float sin_a);
    /**
     * @brief calculate_laser_collision_from_circle Calculate the lidar collision from a circle.
     * @param radius Radius of the circle.
     * @param cx X position of the circle.
     * @param cy Y position of the circle.
     */
    void calculate_laser_collision_from_circle(const float radius, const float cx, const float cy);


#ifdef USE_AVX
    /****************************************************************
     * AVX PART
     ****************************************************************/
    #include <immintrin.h>

    /**
     * @brief avx_load_laser_x Get the avx register of the vector laser x.
     * @param index Vector index.
     * @return Avx register.
     */
    inline __m256 avx_load_laser_x(const int index)
    {
        return _mm256_loadu_ps(laser_x.data() + index);
    }
    /**
     * @brief avx_load_laser_y Get the avx register of the vector laser y.
     * @param index Vector index.
     * @return Avx register.
     */
    inline __m256 avx_load_laser_y(const int index)
    {
        return _mm256_loadu_ps(laser_y.data() + index);
    }
    /**
     * @brief avx_load_laser_distance Get the avx register of the vector laser distance.
     * @param index Vector index.
     * @return Avx register.
     */
    inline __m256 avx_load_laser_distance(const int index)
    {
        return _mm256_loadu_ps(laser_distance.data() + index);
    }
    /**
     * @brief avx_store_laser_distance Store the avx register in the vector laser distance.
     * @param index Vector index.
     * @param value Avx register
     */
    void avx_store_laser_distance(const int index, __m256 value)
    {
        _mm256_storeu_ps(laser_distance.data() + index, value);
    }

    /**
     * @brief avx_calculate_laser_collision_from_circle Calculate the lidar collision from a circle with avx.
     * @param radius Radius of the circle.
     * @param cx X position of the circle.
     * @param cy Y position of the circle.
     */
    void avx_calculate_laser_collision_from_circle(const float radius, const float cx, const float cy);
    /**
     * @brief avx_calculate_laser_collision_from_line Calculate the lidar collision from a line with avx.
     * @param x1 X posiiton of the first point.
     * @param y1 Y posiiton of the first point.
     * @param x2 X posiiton of the second point.
     * @param y2 Y posiiton of the second point.
     */
    void avx_calculate_laser_collision_from_line(const float px1, const float py1, const float px2, const float py2);

    /**
     * @brief avx_print Print the avx value.
     * @param value Avx register.
     */
    inline void avx_print(__m256 value)
    {
        float array[8];
        _mm256_storeu_ps(array, value);

        std::cout << "avx(";
        for (int i = 0; i < 7; ++i) {
            std::cout << array[i] << ", ";
        }
        std::cout << array[7] << ")" << std::endl;
    }

    /**
     * @brief avx_nxor Nxor for avx.
     * @param a Avx register.
     * @param b Avx register.
     * @return Nxor result.
     */
    inline __m256 avx_nxor(__m256 a, __m256 b)
    {
        return _mm256_xor_ps(_mm256_xor_ps(a, b), _mm256_castsi256_ps(_mm256_set1_epi32(0xFFFFFFFF)));
    }



#endif


public:
    /**
     * @brief Lidar Lidar constructor.
     */
    Lidar();
    /**
     * @brief Lidar Lidar constructor.
     * @param hz Sensor rate in hertz.
     * @param angle_step Lidar angle step.
     * @param angle_min Minimal lidar angle.
     * @param angle_max Maximal lidar angle.
     * @param range_min Minimal lidar range.
     * @param range_max Maximal lidar range.
     * @param bias Bias of the lidar.
     */
    Lidar(float hz, float angle_step, float angle_min, float angle_max, float range_min, float range_max, float bias);
    /**
     * @brief fill_laser_distance_with_range_max Fill the lidar with maximal range.
     */
    void fill_laser_distance_with_range_max();

    /**
     * @brief calculate_laser_collision_from_line Calculate the lidar collision of a line,
     * @param pose Robot pose.
     * @param point1 First line point.
     * @param point2 Second line point.
     */
    void calculate_laser_collision_from_line(const Eigen::Affine2f &pose, Eigen::Vector2f point1, Eigen::Vector2f point2);
    /**
     * @brief calculate_laser_collision_from_circle Calculate the lidar collision of the circle.
     * @param pose Robot pose.
     * @param point Position of the circle.
     * @param radius Radius of the circle.
     */
    void calculate_laser_collision_from_circle(const Eigen::Affine2f &pose, Eigen::Vector2f point, const float radius);
    /**
     * @brief apply_bias Add bias to the lidar distance.
     */
    void apply_bias();


    /**
     * @brief get_angle_min Get the lidar minimum angle.
     * @return Lidar angle minimum.
     */
    float get_angle_min() const;
    /**
     * @brief get_angle_max Get the lidar maximum angle.
     * @return Lidar angle maximum.
     */
    float get_angle_max() const;
    /**
     * @brief get_angle_step Get the lidar angle step.
     * @return Lidar angle step.
     */
    float get_angle_step() const;
    /**
     * @brief get_range_min Get the lidar maximum range.
     * @return Lidar maximum range.
     */
    float get_range_min() const;
    /**
     * @brief get_range_max Get the lidar minimum range.
     * @return Lidar minimum range.
     */
    float get_range_max() const;
    /**
     * @brief get_bias Get the bias rate of the lidar.
     * @return Lidar bias.
     */
    float get_bias() const;
    /**
     * @brief get_hz Rate of the lidar in hertz.
     * @return Lidar hertz.
     */
    float get_hz()const;

    /**
     * @brief print_laser Print the curretn laser distance range.
     */
    void print_laser();

    /***
     *  Inline
     ***/
    /**
     * @brief get_laser_size Get the laser vector size.
     * @return Vector size.
     */
    inline int get_laser_size() const
    {
        return this->laser_distance.size();
    }

    /**
     * @brief get_laser_distance Get the laser distance.
     * @param index Vector index.
     * @return Laser distance.
     */
    inline float get_laser_distance(const int index) const
    {
        return this->laser_distance[index];
    }

    /**
     * @brief get_laser_x_at Get the laser unit vector x.
     * @param index Vector index.
     * @return Laser x.
     */
    inline float get_laser_x_at(const int index) const
    {
        return laser_x[index];
    }

    /**
     * @brief get_laser_y_at Get the laser unit vector Y.
     * @param index Vector index.
     * @return Laser y.
     */
    inline float get_laser_y_at(const int index) const
    {
        return laser_y[index];
    }

    /**
     * @brief get_laser_x_distance_at Get the laser unit vector x with distance scaled.
     * @param index Vector index.
     * @return Laser x scaled.
     */
    inline float get_laser_x_distance_at(const int index) const
    {
        return laser_x[index] * laser_distance[index];
    }

    /**
     * @brief get_laser_y_distance_at Get the laser unit vector y with distance scaled.
     * @param index Vector index.
     * @return Laser y scaled.
     */
    inline float get_laser_y_distance_at(const int index) const
    {
        return laser_y[index] * laser_distance[index];
    }

    /**
     * @brief get_laser_distance_normalized Get the laser distance normalized of the maximal lidar range.
     * @param index Vector index.
     * @return Normalized laser range.
     */
    inline float get_laser_distance_normalized(const int index) const
    {
        return laser_distance[index] / range_max;
    }

    /**
     * @brief get_laser_point_at Get the laser unit vector as vector.
     * @param index Vector index.
     * @return Laser unit vector.
     */
    inline Eigen::Vector2f get_laser_point_at(const int index) const
    {
        return Eigen::Vector2f(laser_x[index], laser_y[index]);
    }

    /**
     * @brief get_laser_point_distance_at Get the laser unit vector scaled with distance as vector.
     * @param index Vector index.
     * @return Laser unit vector scaled distance.
     */
    inline Eigen::Vector2f get_laser_point_distance_at(const int index) const
    {
        return Eigen::Vector2f(laser_x[index], laser_y[index]) * laser_distance[index];
    }
};
