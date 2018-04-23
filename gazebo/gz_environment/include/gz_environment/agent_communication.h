#ifndef AGENT_CONNECTION_H
#define AGENT_CONNECTION_H

#include "gz_environment/status_helper.h"
#include "gz_environment/gazebo_communication.h"
#include "msg_to_environment.pb.h"

#include <boost/asio.hpp>
#include <boost/asio/error.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>


class AgentCommunication
{
private:
    boost::signals2::signal<void (std::shared_ptr<MsgToEnvironment>)> signal_process_msg_to_environment;

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::endpoint endpoint;
    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::ip::tcp::socket socket;

    char * buffer_read_msg_length;
    char * buffer_read_msg;
    char * buffer_write_msg;

public:

    AgentCommunication(unsigned short port);
    ~AgentCommunication();
    void run();
    void handler_accept(const boost::system::error_code & error);
    void handler_read_msg_to_environment(const boost::system::error_code & error, const size_t &bytes_transferred);
    void handler_read_msg_to_environment_length(const boost::system::error_code & error, const size_t &bytes_transferred);
    void handler_write_msg_to_agent(const boost::system::error_code &error, const size_t &bytes_transferred);
    void read_msg_to_environment_length();
    void read_msg_to_environment(unsigned int msg_length);
    void write_msg_to_agent(std::shared_ptr<MsgToAgent> msg);
    void process_msgs_to_agent(std::shared_ptr<MsgToAgent> msg);
    boost::signals2::signal<void (std::shared_ptr<MsgToEnvironment>)> &get_signal_process_msg_to_environment();
};

#endif //AGENT_CONNECTION_H
