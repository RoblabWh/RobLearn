#pragma once

#include <eigen3/Eigen/Core>

class Line
{
private:
    Eigen::Vector2f point1;
    Eigen::Vector2f point2;
public:
    Line();
    Line(float x1, float y1, float x2, float y2);
    Line(Eigen::Vector2f point1, Eigen::Vector2f point2);

    void set_point1(Eigen::Vector2f point);
    void set_point1(float x, float y);
    void set_point2(Eigen::Vector2f point);
    void set_point2(float x, float y);

    Eigen::Vector2f get_point1() const;
    Eigen::Vector2f get_point2() const;

    float get_point1_x() const;
    float get_point1_y() const;
    float get_point2_x() const;
    float get_point2_y() const;
};
