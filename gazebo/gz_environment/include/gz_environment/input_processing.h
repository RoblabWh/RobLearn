#ifndef INPUT_PROCESSING_H
#define INPUT_PROCESSING_H

#include <gz_environment/msg_to_agent.pb.h>

#include <gazebo/msgs/msgs.hh>

/**
 * @brief The InputProcessing class This class process the laserscan message of the gazebo simulation and calculate the reward of the fitness function.
 */
class InputProcessing
{
private:
    /**
     * @brief position_last Position of the last step form the gazebo server.
     */
    gazebo::math::Vector3 position_last;
    /**
     * @brief pose_start Start pose of the turtlebot.
     */
    gazebo::math::Pose pose_start;
    /**
     * @brief pose_target Target pose of the turtlebot.
     */
    gazebo::math::Pose pose_target;

    /**
     * @brief is_collidied Flag if the turtlebot is collieded.
     */
    bool is_collidied;
    /**
     * @brief is_goal Flag if the turtlebot reached the target.
     */
    bool is_goal;

    /**
     * @brief check_collision Check if the turtelbot is collied.
     * @param laserscan Laserscan which is check the collision.
     * @return True when the laserscanrange is to small.
     */
    bool check_collision(ConstLaserScanStampedPtr laserscan);
    /**
     * @brief check_goal Check if the turtlebot reach the target.
     * @param laserscan Laserscan which is check the target by using the pose of the laserscan.
     * @return True if the turtlebot reach the target.
     */
    bool check_goal(ConstLaserScanStampedPtr laserscan);
    /**
     * @brief get_observacation Convert the laserscan message to the observation for the python agent.
     * @param laserscan Input laserscan.
     * @param msg Message for the python agent.
     */
    void get_observacation(ConstLaserScanStampedPtr laserscan, std::shared_ptr<MsgToAgent> msg);
    /**
     * @brief get_reward Get the reward from pose of the laserscan message.
     * @param laserscan Input laserscan.
     * @return Reward value.
     */
    float get_reward(ConstLaserScanStampedPtr laserscan);
    /**
     * @brief get_pose Get the pose from the laserscan message.
     * @param laserscan Input laserscan.
     * @return Pose of the laserscanner.
     */
    gazebo::math::Pose get_pose(ConstLaserScanStampedPtr laserscan);
    /**
     * @brief quaternion_from_two_vectors Calculate the Quaternion from to vector.
     * @param u First vector.
     * @param v Second vector.
     * @return Quaternion.
     */
    gazebo::math::Quaternion quaternion_from_two_vectors(gazebo::math::Vector3 u, gazebo::math::Vector3 v);
public:
    /**
     * @brief InputProcessing Constructer of the InputProcessing.
     */
    InputProcessing();
    /**
     * @brief process Process the lasescan message to the message for the python agent.
     * @param laserscan Input laserscan.
     * @return  Message to the agent.
     */
    std::shared_ptr<MsgToAgent> process(ConstLaserScanStampedPtr laserscan);

    /**
     * @brief get_pose_start Get the start pose.
     * @return Start pose.
     */
    gazebo::math::Pose get_pose_start();
    /**
     * @brief get_pose_target Get target pose.
     * @return Target pose.
     */
    gazebo::math::Pose get_pose_target();
};

#endif //INPUT_PROCESSING_H
