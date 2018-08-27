#pragma once

#include <eigen3/Eigen/Core>
#include <iostream>

/**
 * @brief The Line class This class represent a line by to points.
 */
class Line
{
private:
    /**
     * @brief point1 First line point.
     */
    Eigen::Vector2f point1;
    /**
     * @brief point2 Second line point.
     */
    Eigen::Vector2f point2;
public:
    /**
     * @brief Line Constructor of the line.
     */
    Line();
    /**
     * @brief Line Constructor of the line.
     * @param x1 X position of the first points.
     * @param y1 Y position of the first points.
     * @param x2 X position of the second points.
     * @param y2 Y position of the second points.
     */
    Line(float x1, float y1, float x2, float y2); /**
     * @brief Line Constructor of the line.
     * @param point1 Frist point.
     * @param point2 Second point.
     */
    Line(Eigen::Vector2f point1, Eigen::Vector2f point2);

    /**
     * @brief set_point1 Set the first point.
     * @param point Vector as first point.
     */
    void set_point1(Eigen::Vector2f point);
    /**
     * @brief set_point1 Set the first point.
     * @param x X position of the first point.
     * @param y Y position of the first point.
     */
    void set_point1(float x, float y);
    /**
     * @brief set_point2 Set the second point.
     * @param point VEctor as second point.
     */
    void set_point2(Eigen::Vector2f point);
    /**
     * @brief set_point2 Set the second point.
     * @param x X position of the second point.
     * @param y Y position of the second point.
     */
    void set_point2(float x, float y);

    /**
     * @brief get_point1 Get the fist point as vector.
     * @return Vector as point.
     */
    Eigen::Vector2f get_point1() const;
    /**
     * @brief get_point2 Get the second point as vector,
     * @return Vector as point.
     */
    Eigen::Vector2f get_point2() const;

    /**
     * @brief get_point1_x Get x position of the first point.
     * @return X position of the first point.
     */
    float get_point1_x() const;
    /**
     * @brief get_point1_y Get y position of the first point.
     * @return Y position of the first point.
     */
    float get_point1_y() const;
    /**
     * @brief get_point2_x Get x position of the second point.
     * @return X position of the second point.
     */
    float get_point2_x() const;
    /**
     * @brief get_point2_y Get y position of the second point.
     * @return Y position of the second point.
     */
    float get_point2_y() const;

    /**
     * @brief operator << Operator for cout.
     * @param os Ostream.
     * @param c Line.
     * @return Ostream.
     */
    friend std::ostream& operator<<(std::ostream &os, const Line& l);
};
