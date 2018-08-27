#ifndef AGENT_CONNECTION_H
#define AGENT_CONNECTION_H

#include "gz_environment/status_helper.h"
#include "gz_environment/gazebo_communication.h"
#include "msg_to_environment.pb.h"

#include <boost/asio.hpp>
#include <boost/asio/error.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

/**
 * @brief The AgentCommunication class This class provide the Communcation to the python agent.
 */
class AgentCommunication
{
private:
    /**
     * @brief signal_process_msg_to_environment Signal from the GazeboCommunication to send the observation to the python agent.
     */
    boost::signals2::signal<void (std::shared_ptr<MsgToEnvironment>)> signal_process_msg_to_environment;

    /**
     * @brief io_service Asio io service hander for the network communication to the py agent.
     */
    boost::asio::io_service io_service;
    /**
     * @brief endpoint Configuration of the tcp connection to the python agent.
     */
    boost::asio::ip::tcp::endpoint endpoint;
    /**
     * @brief acceptor Listen to a connection from the python agent.
     */
    boost::asio::ip::tcp::acceptor acceptor;
    /**
     * @brief socket Socket for the python agent.
     */
    boost::asio::ip::tcp::socket socket;

    /**
     * @brief buffer_read_msg_length Buffer for the length of the incomming message.
     */
    char * buffer_read_msg_length;
    /**
     * @brief buffer_read_msg Buffer for the contnent of the incomming message.
     */
    char * buffer_read_msg;
    /**
     * @brief buffer_write_msg Buffer for the outgoing message.
     */
    char * buffer_write_msg;

    /**
     * @brief is_terminated Flag for the termination of the communication.
     */
    bool is_terminated;

    /**
     * @brief reconnecting Reconnecting to the python agent.
     */
    void reconnecting();

public:

    /**
     * @brief AgentCommunication Contructor of the class. Call run method to start the communication.
     * @param port Port number to communication to the python agent.
     */
    AgentCommunication(unsigned short port);
    ~AgentCommunication();
    /**
     * @brief run Start the communication to the python agent.
     */
    void run();
    /**
     * @brief handler_accept Handler for the when a communication is established.
     * @param error Error code.
     */
    void handler_accept(const boost::system::error_code & error);
    /**
     * @brief handler_read_msg_to_environment Handler for the incomming message when the lenght is known. Read the content of the incoming message.
     * @param error Error code.
     * @param bytes_transferred Number of the transfered bytes.
     */
    void handler_read_msg_to_environment(const boost::system::error_code & error, const size_t &bytes_transferred);
    /**
     * @brief handler_read_msg_to_environment_length Handler for the incomming message. Read the length of the content from the incomming message.
     * @param error Error code.
     * @param bytes_transferred Number of the transfered bytes.
     */
    void handler_read_msg_to_environment_length(const boost::system::error_code & error, const size_t &bytes_transferred);
    /**
     * @brief handler_write_msg_to_agent Handler for the outgoing message.
     * @param error Error code.
     * @param bytes_transferred Number of the transfered bytes.
     */
    void handler_write_msg_to_agent(const boost::system::error_code &error, const size_t &bytes_transferred);
    /**
     * @brief read_msg_to_environment_length Read the length of the incomming message.
     */
    void read_msg_to_environment_length();
    /**
     * @brief read_msg_to_environment Read the content of the incomming message.
     * @param msg_length
     */
    void read_msg_to_environment(unsigned int msg_length);
    /**
     * @brief write_msg_to_agent Write the message to the agent.
     * @param msg Message to the python agent.
     */
    void write_msg_to_agent(std::shared_ptr<MsgToAgent> msg);
    /**
     * @brief process_msgs_to_agent Process the msgs to agent.
     * @param msg Message to the python agent.
     */
    void process_msgs_to_agent(std::shared_ptr<MsgToAgent> msg);
    /**
     * @brief terminated Check if the communication is terminated.
     * @return True if communication is terminated.
     */
    bool terminated();
    /**
     * @brief get_signal_process_msg_to_environment Get the signal for binding.
     * @return Signal.
     */
    boost::signals2::signal<void (std::shared_ptr<MsgToEnvironment>)> &get_signal_process_msg_to_environment();
};

#endif //AGENT_CONNECTION_H
