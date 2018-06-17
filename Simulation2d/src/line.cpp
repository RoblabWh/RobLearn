#include "simulation2d/line.h"

Line::Line():point1(0,0), point2(0,0)
{

}

Line::Line(float x1, float y1, float x2, float y2):point1(x1,y1), point2(x2,y2)
{

}

Line::Line(Eigen::Vector2f point1, Eigen::Vector2f point2): point1(point1), point2(point2)
{

}

Eigen::Vector2f Line::get_point1() const
{
    return this->point1;
}

Eigen::Vector2f Line::get_point2() const
{
    return this->point2;
}

float Line::get_point1_x() const
{
    return this->point1.x();
}

float Line::get_point1_y() const
{
    return this->point1.y();
}

float Line::get_point2_x() const
{
    return this->point2.x();
}

float Line::get_point2_y() const
{
    return this->point2.y();
}

void Line::set_point1(Eigen::Vector2f point)
{
    this->point1 = point;
}

void Line::set_point2(Eigen::Vector2f point)
{
    this->point2 = point;
}

void Line::set_point1(float x, float y)
{
    this->point1(x,y);
}


void Line::set_point2(float x, float y)
{
    this->point2(x,y);
}

std::ostream& operator<<(std::ostream &os, const Line& l)
{
    return os << "Line(x1=" << l.get_point1_x() << ", y1=" << l.get_point1_y() << ", x2=" << l.get_point2_x() << ", y2=" << l.get_point2_y() << ")";
}
