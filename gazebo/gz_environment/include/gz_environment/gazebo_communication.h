#ifndef GAZEBO_COMMUNICATION_H
#define GAZEBO_COMMUNICATION_H

#include "gz_environment/msg_to_agent.pb.h"
#include "gz_environment/msg_to_environment.pb.h"
#include "gz_environment/input_processing.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo/msgs/msgs.hh>

#include <boost/signals2.hpp>

/**
 * @brief The GazeboCommunication class This class orgnized the communication to the gazebo server.
 */
class GazeboCommunication
{
public:
    /**
     * @brief The STATE_LASER enum State of the laser scan of the server.
     */
    enum class STATE_LASER{UNSYNC, SYNCING, SYNCED};
private:
    /**
     * @brief input_processing Process the laserscan data and calculate the fitness function.
     */
    InputProcessing input_processing;

    /**
     * @brief node Gazebo node for the communication to the gazebo server.
     */
    gazebo::transport::NodePtr node;

    /**
     * @brief publisher_world_control Publisher to control the world of the gazebo server.
     */
    gazebo::transport::PublisherPtr publisher_world_control;
    /**
     * @brief publisher_turtlebot2_velocity_cmd Publsher to send the velocity command of the turtle bot in gazebo.
     */
    gazebo::transport::PublisherPtr publisher_turtlebot2_velocity_cmd;
    /**
     * @brief publisher_turtlebot2_model_modify Publser to modify the turtlebot position.
     */
    gazebo::transport::PublisherPtr publisher_turtlebot2_model_modify;

    /**
     * @brief subscriber_laserscan Subscriber of the turtlebot laserscanner.
     */
    gazebo::transport::SubscriberPtr subscriber_laserscan;

    /**
     * @brief address Address gazebo server.
     */
    std::string address;
    /**
     * @brief port Port number gazebo server.
     */
    unsigned int port;

    /**
     * @brief signal_process_msg_to_agent Signal from the AgentCommunication to send the action to the gazebo server.
     */
    boost::signals2::signal<void (std::shared_ptr<MsgToAgent>)> signal_process_msg_to_agent;

    /**
     * @brief state_laser State of the laserscan.
     */
    GazeboCommunication::STATE_LASER state_laser;

    /**
     * @brief initialised Flag if the commincation to the gazebo server is initilazied.
     */
    bool initialised;
    /**
     * @brief cycles_to_start_position How many times the message is published to set the turtlebot to the start position.
     */
    int cycles_to_start_position;

    /**
     * @brief iterations Number of the iteration from the gazebo server
     */
    unsigned long iterations = 0;
    /**
     * @brief resets Number of the resets from the gazebo server.
     */
    unsigned long resets = 0;

public:
    /**
     * @brief GazeboCommunication Constructer of the GazeboCommunication. To start the communication to the cazebo server init musst be called.
     */
    GazeboCommunication();
    ~GazeboCommunication();
    /**
     * @brief process_msg_to_environment Process the message to enviromnent of the gazebo server.
     * @param msg Message to the environmet.
     */
    void process_msg_to_environment(std::shared_ptr<MsgToEnvironment> msg);
    /**
     * @brief init Initialization to the communication to the gazeob server.
     * @param argc Number of the arguments.
     * @param argv Argmuents values.
     * @return True if the Communication is successful.
     */
    bool init(int argc, char *argv[]);
    /**
     * @brief subscriber_callback_laser Subscriber callback for the laserscan message from the gazebo server.
     * @param laserscan Laserscan message.
     */
    void subscriber_callback_laser(ConstLaserScanStampedPtr &laserscan);
    /**
     * @brief send_velocity_cmd Send the velocity command to the gazebo server.
     * @param linear_velocity Linear velocity of the turtlebot.
     * @param angular_velocity Angular velotcity of the turtlebot.
     */
    void send_velocity_cmd(const double &linear_velocity, const double &angular_velocity);
    /**
     * @brief send_world_control_pause Send the message to pause the gazebo server simulation.
     */
    void send_world_control_pause();
    /**
     * @brief send_world_control_resume Send the messagt to resume the gazebo server simulation.
     */
    void send_world_control_resume();
    /**
     * @brief send_world_control_steps Send the message to the gazebo server to simulate a number of steps.
     * @param steps Number of steps to simulate.
     */
    void send_world_control_steps(int steps);
    /**
     * @brief send_model_modify_turtlebot2_pose Send the message to the gazebo server to set the position of the turtlebot.
     * @param pose Pose message for the position of the turtlebot.
     */
    void send_model_modify_turtlebot2_pose(const gazebo::math::Pose &pose);
    /**
     * @brief is_initialised Check if the gazebo communication is initialzed.
     * @return True if initialized.
     */
    bool is_initialised();
    /**
     * @brief get_port Get the port number of the communication to the gazebo server.
     * @return Port number to the gazebo server.
     */
    unsigned short get_port();
    /**
     * @brief set_state_laser Set the laserscanner state.
     * @param state Laserscanner state.
     */
    void set_state_laser(const GazeboCommunication::STATE_LASER state);
    /**
     * @brief get_state_laser Get the current laserscanner state.
     * @return Laserscanner state.
     */
    STATE_LASER get_state_laser();
    /**
     * @brief get_address Get the ip address to the gazebo server.
     * @return Ip address gazebo server.
     */
    std::string get_address();
    /**
     * @brief get_signal_process_msg_to_agent Get the signal for binding.
     * @return Signal.
     */
    boost::signals2::signal<void (std::shared_ptr<MsgToAgent>)> &get_signal_process_msg_to_agent();
};

#endif //GAZEBO_COMMUNICATION_H
