#pragma once

#include <eigen3/Eigen/Core>
#include <iostream>

/**
 * @brief The Circle class This class represent a circle.
 */
class Circle
{
private:
    /**
     * @brief point Point as position of the circle.
     */
    Eigen::Vector2f point;
    /**
     * @brief radius Radius of the circle.
     */
    float radius;
public:
    /**
     * @brief Circle Constructor of the circle.
     */
    Circle();
    /**
     * @brief Circle Constructor of the circle.
     * @param x X position.
     * @param y Y position.
     * @param radius Radius.
     */
    Circle(const float x, const float y, const float radius);
    /**
     * @brief Circle Constructor of the circle.
     * @param point Point as position.
     * @param radius Radius.
     */
    Circle(Eigen::Vector2f point, float radius);

    /**
     * @brief set_point Set the point as position.
     * @param point Point as position.
     */
    void set_point(const Eigen::Vector2f point);
    /**
     * @brief set_point Set point as position.
     * @param x X position.
     * @param y Y position.
     */
    void set_point(float x, float y);
    /**
     * @brief set_radius Set the radius.
     * @param radius Radius.
     */
    void set_radius(float radius);

    /**
     * @brief get_point Get the point as position.
     * @return Point as position.
     */
    Eigen::Vector2f get_point() const;
    /**
     * @brief get_x Get the x position.
     * @return X position.
     */
    float get_x() const;
    /**
     * @brief get_y Get the y psoition.
     * @return Y position.
     */
    float get_y() const;
    /**
     * @brief get_radius Get the radius.
     * @return Radius.
     */
    float get_radius() const;

    /**
     * @brief operator << Operator for cout.
     * @param os Ostream.
     * @param c Circle.
     * @return Ostream.
     */
    friend std::ostream& operator<<(std::ostream &os, const Circle& c);
};

