#pragma once

#include "simulation2d/data_container.h"
#include "simulation2d/robot.h"
#include "simulation2d/visualization_gnuplot.h"


/**
 * @brief The Simulation2D class This class represent the 2d simulation.
 */
class Simulation2D
{
private:
    /**
     * @brief data Datacontainer for the world representation.
     */
    DataContainer data;
    /**
     * @brief robot Robot of the simulation.
     */
    Robot robot;
    /**
     * @brief visualization Vizualition with gnuplot.
     */
    Visualization_Gnuplot visualization;


    /**
     * @brief time_step_check_collision Timestep when the robot collision must be checked.
     */
    float time_step_check_collision;
    /**
     * @brief time_step_check_laser Timesteo when the collision must be checked.
     */
    float time_step_check_laser;

    /**
     * @brief collision Flag when the robot is collied.
     */
    bool collision;

    /**
     * @brief calculate_iteration Calculate the collision timestemp.
     */
    void calculate_iteration();
    /**
     * @brief load_default_world Load the default world.
     */
    void load_default_world();
    /**
     * @brief load_world Load the world from world file.
     * @param world Word filename.
     * @return True if succefful.
     */
    bool load_world(const std::string &world);

public:
    /**
     * @brief Simulation2D Constructor of Simulation2D.
     */
    Simulation2D();
    /**
     * @brief init Init the world.
     * @param world World file name.
     * @return True if successful.
     */
    bool init(std::string world);
    /**
     * @brief step Make a step in the simulation until the lidar collision check.
     * @param linear_velocity Linear velocity of the robot.
     * @param angular_velocity Angular velocity of the robot,
     * @param skip_number Number of lidar skip until return.
     */
    void step(const float linear_velocity, const float angular_velocity, int skip_number=1);
    /**
     * @brief get_robot_pose Get the robot pose.
     * @return Robot pose.
     */
    Eigen::Affine2f get_robot_pose();
    /**
     * @brief set_robot_pose Set the robot pose.
     * @param x X position.
     * @param y Y position.
     * @param orientation Orientation in radiant.
     */
    void set_robot_pose(const float x, const float y, const float orientation);
    /**
     * @brief visualize Visual the current scene with gnuplot.
     * @param end_x Target node position x.
     * @param end_y Target node position y.
     * @param end_radius Target node radius.
     */
    void visualize(const float end_x, const float end_y, const float end_radius);


    /**
     * @brief observation_size Observation size from the lidar.
     * @return Observation size.
     */
    inline int observation_size() const
    {
        return robot.get_lidar().get_laser_size();
    }
    /**
     * @brief observation_at Get the observation from the vector.
     * @param index Vector index.
     * @return Lidar value.
     */
    inline float observation_at(const int index) const
    {
        return robot.get_lidar().get_laser_distance_normalized(index);
    }

    /**
     * @brief get_robot_pose_x Get the robot position x.
     * @return Robot position x.
     */
    inline float get_robot_pose_x() const
    {
        return robot.get_position_x();
    }

    /**
     * @brief get_robot_pose_y Get the robot position y.
     * @return Robot position y.
     */
    inline float get_robot_pose_y() const
    {
        return robot.get_position_y();
    }

    /**
     * @brief get_robot_pose_orientation Get the robot orientation in radiant.
     * @return orienatation in radiant.
     */
    inline float get_robot_pose_orientation() const
    {
        return robot.get_orientation();
    }

    /**
     * @brief done Check when the robot is collidied.
     * @return True if robot is collidied.
     */
    inline bool done() const
    {
        return this->collision;
    }

    /**
     * @brief observation_min_clustered_at Get the observation in clustered segments and return the minimum lidar size..
     * @param index Index vector.
     * @param cluster_size Cluster size of the segments.
     * @return Minimum laser value.
     */
    inline float observation_min_clustered_at(int index, const int cluster_size) const
    {
        float value = 1.0f;
        index *= cluster_size;
        int index_end = std::min(index + cluster_size, robot.get_lidar().get_laser_size());

        while(index < index_end)
        {
            value = std::min(value, robot.get_lidar().get_laser_distance_normalized(index));
            index++;
        }

        return value;
    }

    /**
     * @brief observation_min_clustered_size Get the clustered segments vector size.
     * @param cluster_size CCluster size of the segments.
     * @return Vector size.
     */
    inline int observation_min_clustered_size(const int cluster_size) const
    {
        return robot.get_lidar().get_laser_size() / cluster_size;
    }

};


