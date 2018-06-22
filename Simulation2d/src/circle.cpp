#include "simulation2d/circle.h"

Circle::Circle():point(0,0),radius(0)
{

}

Circle::Circle(const Eigen::Vector2f point, const float radius):point(point), radius(radius)
{

}

Circle::Circle(const float x, const float y, const float radius):point(x,y), radius(radius)
{

}

void Circle::set_point(const Eigen::Vector2f point)
{
    this->point = point;
}

void Circle::set_point(float x, float y)
{
    this->point(x,y);
}

void Circle::set_radius(float radius)
{
    this->radius = radius;
}

Eigen::Vector2f Circle::get_point() const
{
    return this->point;
}

float Circle::get_x() const
{
    return this->point.x();
}

float Circle::get_y() const
{
    return this->point.y();
}

float Circle::get_radius() const
{
    return this->radius;
}

std::ostream& operator<<(std::ostream &os, const Circle& c)
{
    return os << "Circle(x=" << c.get_x() << ", y=" << c.get_y() << ", r=" << c.get_radius() << ")";
}
