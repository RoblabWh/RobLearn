#pragma once

#include "simulation2d/line.h"
#include "simulation2d/circle.h"
#include "simulation2d/robot.h"
#include<immintrin.h>

#include<vector>
#include<iostream>




class DataContainer
{
private:
    int avx_line_size;
    int avx_circle_size;

    std::vector<float> line_x1;
    std::vector<float> line_y1;
    std::vector<float> line_x2;
    std::vector<float> line_y2;
    std::vector<float> line_distance;

    std::vector<float> circle_x;
    std::vector<float> circle_y;
    std::vector<float> circle_radius;
    std::vector<float> circle_distance;

    void line_clear();
    void line_reserve(int size);
    void circle_clear();
    void circle_reserve(int size);
    int calculate_avx_size(int size);

    void calculate_line_distance_from_point(const float x, const float y);

    void calculate_circle_distance_from_point(const float x,const  float y);

    void calculate_laser_collision(const Robot &robot);

    /***
     * Inline
     ***/
    inline float get_line_distance_at(const int index) const
    {
        return line_distance[index];
    }

    inline bool is_point_in_line_segment(float x, float y, int line_index)
    {
        // from: https://stackoverflow.com/questions/17581738/check-if-a-point-projected-on-a-line-segment-is-not-outside-it
        float dx = line_x2[line_index] - line_x1[line_index];
        float dy = line_y2[line_index] - line_y1[line_index];
        float inner_product = (x - line_x1[line_index]) * dx + (y - line_y1[line_index]) * dy;
        return 0 <= inner_product && inner_product <= dx*dx + dy*dy;
    }

    /***
     * AVX PART
     ***/
#ifdef USE_AVX

    void avx_calculate_line_distance_from_point(const float x, const float y);
    void avx_calculate_circle_distance_from_point(const float x, const float y);
    inline __m256 avx_load_line_x1(const int index)
    {
        return _mm256_loadu_ps(line_x1.data() + index);
    }
    inline __m256 avx_load_line_y1(const int index)
    {
        return _mm256_loadu_ps(line_y1.data() + index);
    }
    inline __m256 avx_load_line_x2(const int index)
    {
        return _mm256_loadu_ps(line_x2.data() + index);
    }
    inline __m256 avx_load_line_y2(const int index)
    {
        return _mm256_loadu_ps(line_y2.data() + index);
    }
    inline void avx_store_line_distance(const int index, const __m256 value)
    {
        _mm256_storeu_ps(line_distance.data() + index, value);
    }
    inline __m256 avx_load_circle_x(const int index)
    {
        return _mm256_loadu_ps(circle_x.data() + index);
    }
    inline __m256 avx_load_circle_y(const int index)
    {
        return _mm256_loadu_ps(circle_y.data() + index);
    }
    inline void avx_store_circle_distance(const int index, const __m256 value)
    {
        _mm256_storeu_ps(circle_distance.data() + index, value);
    }
    inline __m256 avx_abs(__m256 value)
    {
        static const __m256 SIGNMASK = _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));
        return _mm256_andnot_ps(SIGNMASK, value);
    }
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

#endif

public:
    DataContainer();

    void set_lines(const std::vector<Line> &lines);
    void set_circles(const std::vector<Circle> & circles);
    bool calculate_robot_collision(Robot & robot);
    void calculate_lidar_collision(Robot &robot);

    /***
     * Inline function
     ***/
    inline const int get_line_size() const
    {
        return line_distance.size();
    }

    inline Line get_line_at(const int index) const
    {
        return Line(line_x1[index],line_y1[index],line_x2[index],line_y2[index]);
    }

    inline Eigen::Vector2f get_line_point1_at(const int index) const
    {
        return Eigen::Vector2f(line_x1[index], line_y1[index]);
    }

    inline Eigen::Vector2f get_line_point2_at(const int index) const
    {
        return Eigen::Vector2f(line_x2[index], line_y2[index]);
    }

    inline const int get_circle_size() const
    {
        return this->circle_distance.size();
    }

    inline Circle get_circle_at(const int index) const
    {
        return Circle(circle_x[index], circle_y[index], circle_radius[index]);
    }

    inline Eigen::Vector2f get_circle_point_at(const int index) const
    {
        return Eigen::Vector2f(circle_x[index], circle_y[index]);
    }

    inline float get_circle_radius_at(const int index) const
    {
        return circle_radius[index];
    }

    inline float get_line_x1_at(const int index) const
    {
        return line_x1[index];
    }
    inline float get_line_y1_at(const int index) const
    {
        return line_y1[index];
    }
    inline float get_line_x2_at(const int index) const
    {
        return line_x2[index];
    }
    inline float get_line_y2_at(const int index) const
    {
        return line_y2[index];
    }
    inline float get_circle_x_at(const int index) const
    {
        return circle_x[index];
    }
    inline float get_circle_y_at(const int index) const
    {
        return circle_y[index];
    }
};
