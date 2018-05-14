#pragma once

#include <eigen3/Eigen/Core>

class Circle
{
private:
    Eigen::Vector2f point;
    float radius;
public:
    Circle();
    Circle(const float x, const float y, const float radius);
    Circle(Eigen::Vector2f point, float radius);

    void set_point(const Eigen::Vector2f point);
    void set_point(float x, float y);
    void set_radius(float radius);

    Eigen::Vector2f get_point() const;
    float get_x() const;
    float get_y() const;
    float get_radius() const;
};
