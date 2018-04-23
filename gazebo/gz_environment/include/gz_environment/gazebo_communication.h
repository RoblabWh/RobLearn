#ifndef GAZEBO_COMMUNICATION_H
#define GAZEBO_COMMUNICATION_H

#include "gz_environment/msg_to_agent.pb.h"
#include "gz_environment/msg_to_environment.pb.h"
#include "gz_environment/input_processing.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo/msgs/msgs.hh>

#include <boost/signals2.hpp>




class GazeboCommunication
{
public:

    enum class STATE_LASER{UNSYNC, SYNCING, SYNCED};
private:
    InputProcessing input_processing;

    gazebo::transport::NodePtr node;

    gazebo::transport::PublisherPtr publisher_world_control;
    gazebo::transport::PublisherPtr publisher_turtlebot2_velocity_cmd;
    gazebo::transport::PublisherPtr publisher_turtlebot2_model_modify;

    gazebo::transport::SubscriberPtr subscriber_laserscan;

    std::string address;
    unsigned int port;

    boost::signals2::signal<void (std::shared_ptr<MsgToAgent>)> signal_process_msg_to_agent;

    GazeboCommunication::STATE_LASER state_laser;

    bool initialised;
    int cycles_to_start_position;

public:

    unsigned long iterations = 0;
    unsigned long resets = 0;


    GazeboCommunication();
    ~GazeboCommunication();
    void process_msg_to_environment(std::shared_ptr<MsgToEnvironment> msg);
    bool init(int argc, char *argv[]);
    void subscriber_callback_laser(ConstLaserScanStampedPtr &laserscan);
    void send_velocity_cmd(const double &linear_velocity, const double &angular_velocity);
    void send_world_control_pause();
    void send_world_control_resume();
    void send_world_control_steps(int steps);
    void send_model_modify_turtlebot2_pose(const gazebo::math::Pose &pose);
    void set_laserscan_synchronized();
    bool is_initialised();
    unsigned short get_port();
    void set_state_laser(const GazeboCommunication::STATE_LASER state);
    STATE_LASER get_state_laser();
    std::string get_address();
    boost::signals2::signal<void (std::shared_ptr<MsgToAgent>)> &get_signal_process_msg_to_agent();
};

#endif //GAZEBO_COMMUNICATION_H
