#pragma once

#include "simulation2d/line.h"
#include "simulation2d/circle.h"
#include "simulation2d/robot.h"


#include<vector>
#include<iostream>



/**
 * @brief The DataContainer class Presents all data for the world with lines and circles,
 */
class DataContainer
{
private:
    /**
     * @brief avx_line_size The size of the line vector for the avx.
     */
    int avx_line_size;
    /**
     * @brief avx_circle_size The size of the line vector for the avx.
     */
    int avx_circle_size;

    /**
     * @brief line_x1 Vector of the first point x.
     */
    std::vector<float> line_x1;
    /**
     * @brief line_y1 Vector of the first point y.
     */
    std::vector<float> line_y1;
    /**
     * @brief line_x2 Vector of the second point x.
     */
    std::vector<float> line_x2;
    /**
     * @brief line_y2 Vector of the second point y.
     */
    std::vector<float> line_y2;
    /**
     * @brief line_distance Vector of the line distance to the robot,
     */
    std::vector<float> line_distance;

    /**
     * @brief circle_x Vector of the circle position x.
     */
    std::vector<float> circle_x;
    /**
     * @brief circle_y Vector of the circle position y.
     */
    std::vector<float> circle_y;
    /**
     * @brief circle_radius Vector of the circle radius.
     */
    std::vector<float> circle_radius;
    /**
     * @brief circle_distance Vector of the circle distance to the robot.
     */
    std::vector<float> circle_distance;

    /**
     * @brief area_min_x Minimum x area from the world.
     */
    float area_min_x;
    /**
     * @brief area_min_y Minimum y area from the world.
     */
    float area_min_y;
    /**
     * @brief area_max_x Maximum x area from the world.
     */
    float area_max_x;
    /**
     * @brief area_max_y Maximum y area from the world.
     */
    float area_max_y;

    /**
     * @brief line_clear Clear all line vector.
     */
    void line_clear();
    /**
     * @brief line_reserve Reserve all line vector.
     * @param size Size of the vector.
     */
    void line_reserve(int size);
    /**
     * @brief circle_clear Clear all circle vector.
     */
    void circle_clear();
    /**
     * @brief circle_reserve Reserve all circle vector.
     * @param size Size of the vector.
     */
    void circle_reserve(int size);

    /**
     * @brief set_lines Set the line vector for the world,
     * @param lines Line vector.
     */
    void set_lines(const std::vector<Line> &lines);
    /**
     * @brief set_circles Set the circle vector for the world.
     * @param circles Circle vector.
     */
    void set_circles(const std::vector<Circle> & circles);
    /**
     * @brief set_area Set the area size of the world from the lines and circles.
     */
    void set_area();

    /**
     * @brief calculate_avx_size Calculate the avx size for the vector.
     * @param size Normal size.
     * @return Avx vector size.
     */
    int calculate_avx_size(int size);

    /**
     * @brief calculate_line_distance_from_point Calculate the line distance of a given point.
     * @param x X position.
     * @param y Y position.
     */
    void calculate_line_distance_from_point(const float x, const float y);

    /**
     * @brief calculate_circle_distance_from_point Calculate the circle distance of a given point.
     * @param x X position.
     * @param y Y position.
     */
    void calculate_circle_distance_from_point(const float x,const  float y);

    /**
     * @brief calculate_laser_collision Calculate the laser collsion from the robot.
     * @param robot Robot.
     */
    void calculate_laser_collision(const Robot &robot);




    /***
     * Inline
     ***/
    /**
     * @brief is_point_in_line_segment Check if the point is in the line segment.
     * @param x X position.
     * @param y Y position.
     * @param line_index Line vector index.
     * @return True if the point is in line segment.
     */
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
    #include <immintrin.h>

    /**
     * @brief avx_calculate_line_distance_from_point Calculate the line distance of a given point.
     * @param x X position.
     * @param y Y position.
     */
    void avx_calculate_line_distance_from_point(const float x, const float y);
    /**
     * @brief avx_calculate_circle_distance_from_point Calculate the circle distance of a given point.
     * @param x X position.
     * @param y Y position.
     */
    void avx_calculate_circle_distance_from_point(const float x, const float y);
    /**
     * @brief avx_load_line_x1 Get the avx register of the vector line first point x.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_line_x1(const int index)
    {
        return _mm256_loadu_ps(line_x1.data() + index);
    }
    /**
     * @brief avx_load_line_y1 Get the avx register of the vector line first point y.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_line_y1(const int index)
    {
        return _mm256_loadu_ps(line_y1.data() + index);
    }
    /**
     * @brief avx_load_line_x2 Get the avx register of the vector line second point x.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_line_x2(const int index)
    {
        return _mm256_loadu_ps(line_x2.data() + index);
    }
    /**
     * @brief avx_load_line_y2 Get the avx register of the vector line second point y.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_line_y2(const int index)
    {
        return _mm256_loadu_ps(line_y2.data() + index);
    }
    /**
     * @brief avx_store_line_distance Set the avx register of the vector line distance.
     * @param index Vector index.
     * @param value Avx value.
     */
    inline void avx_store_line_distance(const int index, const __m256 value)
    {
        _mm256_storeu_ps(line_distance.data() + index, value);
    }
    /**
     * @brief avx_load_circle_x Get the avx register of the vector circle x.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_circle_x(const int index)
    {
        return _mm256_loadu_ps(circle_x.data() + index);
    }
    /**
     * @brief avx_load_circle_y Get the avx register of the vector circle y.
     * @param index Vector index.
     * @return Avx value.
     */
    inline __m256 avx_load_circle_y(const int index)
    {
        return _mm256_loadu_ps(circle_y.data() + index);
    }
    /**
     * @brief avx_store_circle_distance Set the avx register of the circle distance.
     * @param index Vector index.
     * @param value Avx value.
     */
    inline void avx_store_circle_distance(const int index, const __m256 value)
    {
        _mm256_storeu_ps(circle_distance.data() + index, value);
    }
    /**
     * @brief avx_abs Absolute of the avx register.
     * @param value Avx value.
     * @return Absolute avx register.
     */
    inline __m256 avx_abs(__m256 value)
    {
        static const __m256 SIGNMASK = _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));
        return _mm256_andnot_ps(SIGNMASK, value);
    }

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

#endif

public:
    /**
     * @brief DataContainer Constructor of the Datacontainer.
     */
    DataContainer();

    /**
     * @brief set_world Set the world with lines and circles.
     * @param lines Vector of lines.
     * @param circles Vector of circles.
     */
    void set_world(const std::vector<Line> &lines, const std::vector<Circle> & circles);

    /**
     * @brief calculate_robot_collision Calculate the robot collision.
     * @param robot Robot,
     * @return True if robot collidied.
     */
    bool calculate_robot_collision(Robot & robot);
    /**
     * @brief calculate_lidar_collision Calculate the lidar collision.
     * @param robot Robot.
     */
    void calculate_lidar_collision(Robot &robot);

    /***
     * Inline function
     ***/
    /**
     * @brief get_line_size Get the vector line size.
     * @return Vector size.
     */
    inline const int get_line_size() const
    {
        return line_distance.size();
    }

    /**
     * @brief get_line_at Get the line from line vector.
     * @param index Vector index.
     * @return Line.
     */
    inline Line get_line_at(const int index) const
    {
        return Line(line_x1[index],line_y1[index],line_x2[index],line_y2[index]);
    }

    /**
     * @brief get_line_point1_at Get the frist line point from line vector.
     * @param index Vector index.
     * @return Point as vector.
     */
    inline Eigen::Vector2f get_line_point1_at(const int index) const
    {
        return Eigen::Vector2f(line_x1[index], line_y1[index]);
    }

    /**
     * @brief get_line_point2_at Get the second line point from line vector.
     * @param index Vector index.
     * @return Point as vector.
     */
    inline Eigen::Vector2f get_line_point2_at(const int index) const
    {
        return Eigen::Vector2f(line_x2[index], line_y2[index]);
    }

    /**
     * @brief get_circle_size Get the circle vector size.
     * @return vector size.
     */
    inline const int get_circle_size() const
    {
        return this->circle_distance.size();
    }

    /**
     * @brief get_circle_at Get the circle from circle vector.
     * @param index Vector index.
     * @return Circle.
     */
    inline Circle get_circle_at(const int index) const
    {
        return Circle(circle_x[index], circle_y[index], circle_radius[index]);
    }

    /**
     * @brief get_circle_point_at Get the circle point from circle vector.
     * @param index Vector index.
     * @return Point as vector.
     */
    inline Eigen::Vector2f get_circle_point_at(const int index) const
    {
        return Eigen::Vector2f(circle_x[index], circle_y[index]);
    }

    /**
     * @brief get_circle_radius_at Get the circle radius from circle vector.
     * @param index Vector index.
     * @return Radius.
     */
    inline float get_circle_radius_at(const int index) const
    {
        return circle_radius[index];
    }

    /**
     * @brief get_line_x1_at Get the first point x from line vector.
     * @param index Vector index.
     * @return Frist point x.
     */
    inline float get_line_x1_at(const int index) const
    {
        return line_x1[index];
    }

    /**
     * @brief get_line_y1_at Get the first point y from line vector.
     * @param index Vector index.
     * @return Frist point y.
     */
    inline float get_line_y1_at(const int index) const
    {
        return line_y1[index];
    }
    /**
     * @brief get_line_x2_at Get the second point x from line vector.
     * @param index Vector index.
     * @return Second point x.
     */
    inline float get_line_x2_at(const int index) const
    {
        return line_x2[index];
    }
    /**
     * @brief get_line_y2_at Get the second point y from line vector.
     * @param index Vector index.
     * @return Second point y.
     */
    inline float get_line_y2_at(const int index) const
    {
        return line_y2[index];
    }
    /**
     * @brief get_circle_x_at Get the circle point x from circle vector.
     * @param index Vector index.
     * @return Point x.
     */
    inline float get_circle_x_at(const int index) const
    {
        return circle_x[index];
    }
    /**
     * @brief get_circle_y_at Get the circle point y from circle vector.
     * @param index Vector index.
     * @return Point y.
     */
    inline float get_circle_y_at(const int index) const
    {
        return circle_y[index];
    }
    /**
     * @brief get_area_min_x Get the area minimum x.
     * @return Area minimum x.
     */
    inline float get_area_min_x() const
    {
        return area_min_x;
    }
    /**
     * @brief get_area_max_x Get the area maximum x.
     * @return Area maximum x.
     */
    inline float get_area_max_x() const
    {
        return area_max_x;
    }
    /**
     * @brief get_area_min_y Get the area minum y.
     * @return Area minimum y.
     */
    inline float get_area_min_y() const
    {
        return area_min_y;
    }
    /**
     * @brief get_area_max_y Get the area maximum y.
     * @return Area maximum y.
     */
    inline float get_area_max_y() const
    {
        return area_max_y;
    }

};
