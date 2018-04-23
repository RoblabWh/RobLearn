#include "gz_environment/input_processing.h"

#include <cmath>

const static float STD_COLLISION_RANGE = 0.30f;
const static float STD_GOAL_RADIUS = 1.5f;

InputProcessing::InputProcessing()
{
    this->pose_start.rot.SetFromEuler(0,0,-M_PI_2);
    this->pose_start.pos.Set(0,14,0);

    this->pose_target.rot.SetFromEuler(0,0,-M_PI_2);
    this->pose_target.pos.Set(0,-14,0);

    this->position_last = pose_target.pos;

    this->is_collidied = false;
    this->is_goal = false;
}

std::shared_ptr<MsgToAgent> InputProcessing::process(ConstLaserScanStampedPtr laserscan)
{
    std::shared_ptr<MsgToAgent> msg = std::shared_ptr<MsgToAgent>(new MsgToAgent);

    this->is_collidied = this->check_collision(laserscan);
    this->is_goal = this->check_goal(laserscan);


    this->get_observacation(laserscan, msg);
    msg->set_done(is_collidied || is_goal);
    msg->set_reward(this->get_reward(laserscan));
    msg->set_info("nice");

    if (msg->done())
    {
        this->position_last = pose_start.pos;
    }

    return msg;
}

void InputProcessing::get_observacation(ConstLaserScanStampedPtr laserscan, std::shared_ptr<MsgToAgent> msg)
{
    float laserscan_max_range = laserscan->scan().range_max();
    for (int i = 0; i < laserscan->scan().ranges_size(); ++i)
    {
        float value = laserscan->scan().ranges(i);

        value = (value <= 0 || std::isinf(value))? 0 : value / laserscan_max_range;
        msg->add_observation(value);
    }
}

bool InputProcessing::check_collision(ConstLaserScanStampedPtr laserscan)
{
    bool collision = false;
    for (int i = 0; i < laserscan->scan().ranges_size(); ++i)
    {
        if (laserscan->scan().ranges(i) <= STD_COLLISION_RANGE) {
            collision = true;
            break;
        }
    }

    return collision;
}

float InputProcessing::get_reward(ConstLaserScanStampedPtr laserscan)
{
    gazebo::math::Pose pose = this->get_pose(laserscan);
    gazebo::math::Vector3 position = pose.pos;

    float distance_beween_start_and_goal = (pose_target.pos - pose_start.pos).GetLength();

    float distance_betwenn_last_step = (pose_target.pos - position_last).GetLength() - (pose_target.pos - position).GetLength();
    float distance_to_goal = (pose_target.pos - position).GetLength() / distance_beween_start_and_goal;


    float reward = (1 - distance_to_goal) + distance_betwenn_last_step;

    this->position_last = position;

    if (is_collidied)
    {
        reward = - distance_to_goal * 100;
    } else if (is_goal) {
        reward = 100;
    }

    return reward;
}

gazebo::math::Pose InputProcessing::get_pose(ConstLaserScanStampedPtr laserscan)
{
    gazebo::math::Pose pose;

    pose.pos.x = laserscan->scan().world_pose().position().x();
    pose.pos.y = laserscan->scan().world_pose().position().y();
    pose.pos.z = laserscan->scan().world_pose().position().z();

    pose.rot.x = laserscan->scan().world_pose().orientation().x();
    pose.rot.x = laserscan->scan().world_pose().orientation().y();
    pose.rot.x = laserscan->scan().world_pose().orientation().z();
    pose.rot.x = laserscan->scan().world_pose().orientation().w();

    return pose;
}

gazebo::math::Pose InputProcessing::get_pose_start()
{
    return pose_start;
}

gazebo::math::Pose InputProcessing::get_pose_target()
{
    return pose_target;
}
gazebo::math::Quaternion InputProcessing::quaternion_from_two_vectors(gazebo::math::Vector3 u, gazebo::math::Vector3 v)
{
    u = u.Normalize();
    v = v.Normalize();
    gazebo::math::Vector3 w = u.Cross(v);
    gazebo::math::Quaternion q(1.0f + u.Dot(v), w.x, w.y, w.z);
    q.Normalize();
    return q;
}

bool InputProcessing::check_goal(ConstLaserScanStampedPtr laserscan)
{
    gazebo::math::Pose pose = this->get_pose(laserscan);

    return ( pose_target.pos - pose.pos).GetLength() < STD_GOAL_RADIUS;
}
