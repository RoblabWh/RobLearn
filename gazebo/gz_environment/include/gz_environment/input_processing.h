#ifndef INPUT_PROCESSING_H
#define INPUT_PROCESSING_H

#include <gz_environment/msg_to_agent.pb.h>

#include <gazebo/msgs/msgs.hh>

class InputProcessing
{
private:
    gazebo::math::Vector3 position_last;
    gazebo::math::Pose pose_start;
    gazebo::math::Pose pose_target;

    bool is_collidied;
    bool is_goal;

    bool check_collision(ConstLaserScanStampedPtr laserscan);
    bool check_goal(ConstLaserScanStampedPtr laserscan);
    void get_observacation(ConstLaserScanStampedPtr laserscan, std::shared_ptr<MsgToAgent> msg);
    float get_reward(ConstLaserScanStampedPtr laserscan);
    gazebo::math::Pose get_pose(ConstLaserScanStampedPtr laserscan);
    gazebo::math::Quaternion quaternion_from_two_vectors(gazebo::math::Vector3 u, gazebo::math::Vector3 v);
public:
    InputProcessing();
    std::shared_ptr<MsgToAgent> process(ConstLaserScanStampedPtr laserscan);

    gazebo::math::Pose get_pose_start();
    gazebo::math::Pose get_pose_target();

    void reset();
};

#endif //INPUT_PROCESSING_H
