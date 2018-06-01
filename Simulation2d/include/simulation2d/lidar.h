#pragma once

#include <vector>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class Lidar
{
private:
    int avx_laser_size;

    std::vector<float> laser_x;
    std::vector<float> laser_y;
    std::vector<float> laser_distance;

    float hz;
    float angle_min;
    float angle_max;
    float angle_step;
    float range_min;
    float range_max;
    float bias;


    void laser_clear();
    void laser_reserve(int size);

    void create_laser();
    int calculate_avx_size(int size);

    void prepare_laser_rotated(const Eigen::Affine2f &pose);



    void calculate_laser_collision_from_line(const float x1, const float y1, const float x2, const float y2);

    void calculate_laser_collision_from_circle_wolfram(const float r, const float x_lidar, const float y_lidar, const float cos_a, const float sin_a);
    void calculate_laser_collision_from_circle(const float radius, const float cx, const float cy);


#ifdef USE_AVX
    /****************************************************************
     * AVX PART
     ****************************************************************/
    #include <immintrin.h>

    inline __m256 avx_load_laser_x(const int index)
    {
        return _mm256_loadu_ps(laser_x.data() + index);
    }
    inline __m256 avx_load_laser_y(const int index)
    {
        return _mm256_loadu_ps(laser_y.data() + index);
    }
    inline __m256 avx_load_laser_distance(const int index)
    {
        return _mm256_loadu_ps(laser_distance.data() + index);
    }
    inline __m256 avx_store_laser_distance(const int index, __m256 value)
    {
        _mm256_storeu_ps(laser_distance.data() + index, value);
    }

    void avx_calculate_laser_collision_from_circle(const float radius, const float cx, const float cy);
    void avx_calculate_laser_collision_from_line(const float px1, const float py1, const float px2, const float py2);

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

    inline __m256 avx_nxor(__m256 a, __m256 b)
    {
        return _mm256_xor_ps(_mm256_xor_ps(a, b), _mm256_castsi256_ps(_mm256_set1_epi32(0xFFFFFFFF)));
    }



#endif


public:
    Lidar();
    Lidar(float hz, float angle_step, float angle_min, float angle_max, float range_min, float range_max, float bias);
    void fill_laser_distance_with_range_max();

    void calculate_laser_collision_from_line(const Eigen::Affine2f &pose, Eigen::Vector2f point1, Eigen::Vector2f point2);
    void calculate_laser_collision_from_circle(const Eigen::Affine2f &pose, Eigen::Vector2f point, const float radius);
    void apply_bias();


    float get_angle_min() const;
    float get_angle_max() const;
    float get_angle_step() const;
    float get_range_min() const;
    float get_range_max() const;
    float get_range_max_squared() const;
    float get_bias() const;
    float get_hz()const;

    void print_laser();

    /***
     *  Inline
     ***/
    inline int get_laser_size() const
    {
        return this->laser_distance.size();
    }

    inline float get_laser_distance(const int index) const
    {
        return this->laser_distance[index];
    }

    inline float get_laser_x_at(const int index) const
    {
        return laser_x[index];
    }

    inline float get_laser_y_at(const int index) const
    {
        return laser_y[index];
    }

    inline float get_laser_x_distance_at(const int index) const
    {
        return laser_x[index] * laser_distance[index];
    }

    inline float get_laser_y_distance_at(const int index) const
    {
        return laser_y[index] * laser_distance[index];
    }

    inline float get_laser_distance_normalized(const int index) const
    {
        return laser_distance[index] / range_max;
    }

    inline Eigen::Vector2f get_laser_point_at(const int index) const
    {
        return Eigen::Vector2f(laser_x[index], laser_y[index]);
    }

    inline Eigen::Vector2f get_laser_point_distance_at(const int index) const
    {
        return Eigen::Vector2f(laser_x[index], laser_y[index]) * laser_distance[index];
    }



};
