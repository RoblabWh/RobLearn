#include "gz_environment/agent_communication.h"
#include "gz_environment/status_helper.h"
#include "gz_environment/msg_to_environment.pb.h"

#include <byteswap.h>

const static unsigned int STD_MEMORY_ALLOCATED_BUFFER_READ_LENGTH_MSG = 4;
const static unsigned int STD_MEMORY_ALLOCATED_BUFFER_READ_MSG = 256;
const static unsigned int STD_MEMORY_ALLOCATED_BUFFER_WRITE_MSG = 65535;

AgentCommunication::AgentCommunication(unsigned short port):
    endpoint(boost::asio::ip::tcp::v4(), port),
    acceptor(io_service, endpoint),
    socket(io_service)
{
    MESSAGE_WARN("using port: " << port);

    this->buffer_read_msg = new char [STD_MEMORY_ALLOCATED_BUFFER_READ_MSG];
    this->buffer_read_msg_length = new char [STD_MEMORY_ALLOCATED_BUFFER_READ_LENGTH_MSG];
    this->buffer_write_msg = new char [STD_MEMORY_ALLOCATED_BUFFER_WRITE_MSG];

    acceptor.listen();
    acceptor.async_accept(socket,boost::bind(&AgentCommunication::handler_accept,this, boost::asio::placeholders::error));
}

AgentCommunication::~AgentCommunication()
{
    delete this->buffer_read_msg;
    delete this->buffer_read_msg_length;
    delete this->buffer_write_msg;

    socket.close();
    acceptor.close();
    io_service.stop();

    //signal_process_msg_to_environment.disconnect(0);
}

void AgentCommunication::handler_accept(const boost::system::error_code &error)
{
    if(error)
    {
        MESSAGE_ERROR("[AgentCommunication][handler_accept]: " << error.message());
        io_service.stop();
    }
    else {
        acceptor.close();
        this->read_msg_to_environment_length();
    }
}

void AgentCommunication::handler_read_msg_to_environment(const boost::system::error_code& error, const size_t& bytes_transferred)
{
    if (error) {
        MESSAGE_ERROR("[AgentCommunication][handler_read_msg_to_environment]: " << error.message());
        throw error;
    } else {
        std::shared_ptr<MsgToEnvironment> msg(new MsgToEnvironment);
        msg->ParseFromArray(buffer_read_msg, bytes_transferred);


        signal_process_msg_to_environment(msg);
        read_msg_to_environment_length();
    }
}

void AgentCommunication::handler_read_msg_to_environment_length(const boost::system::error_code& error, const size_t& bytes_transferred)
{
    if (error)
    {
        MESSAGE_ERROR("[AgentCommunication][handler_read_msg_to_environment_length]: " << error.message());
        throw error;
    } else  {

        int msg_length;

        memcpy(&msg_length, buffer_read_msg_length, 4);

    #if __BYTE_ORDER == __ORDER_LITTLE_ENDIAN__
        msg_length = bswap_32(msg_length);
    #endif



        this->read_msg_to_environment(msg_length);
    }
}


void AgentCommunication::handler_write_msg_to_agent(const boost::system::error_code& error, const size_t& bytes_transferred)
{
    if (error) {
        MESSAGE_ERROR("[AgentCommunication][handler_write_msg_to_agent]: " << error.message());
        throw error;
    } else {
    }
}

void AgentCommunication::write_msg_to_agent(std::shared_ptr<MsgToAgent> msg)
{
    int msg_length = msg->ByteSize();
    int total_length = msg_length + 4;

    //this->buffer_write_msg = new char[total_length];


#if __BYTE_ORDER == __ORDER_LITTLE_ENDIAN__
    msg_length = bswap_32(msg_length);
#endif


    memcpy(buffer_write_msg, &msg_length, 4);

    msg->SerializePartialToArray(buffer_write_msg + 4, msg->ByteSize());

    boost::asio::async_write(socket, boost::asio::buffer(buffer_write_msg, total_length), boost::bind(&AgentCommunication::handler_write_msg_to_agent, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::read_msg_to_environment_length()
{
    //this->buffer_read_msg_length = new char[4];
    boost::asio::async_read(socket, boost::asio::buffer(buffer_read_msg_length,4), boost::asio::transfer_exactly(4),boost::bind(&AgentCommunication::handler_read_msg_to_environment_length, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::read_msg_to_environment(unsigned int msg_length)
{
    //this->buffer_read_msg = new char[msg_length];
    boost::asio::async_read(socket, boost::asio::buffer(buffer_read_msg, msg_length), boost::asio::transfer_exactly(msg_length),boost::bind(&AgentCommunication::handler_read_msg_to_environment, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::run()
{
    try {
        io_service.run();
    } catch (boost::system::error_code ec){
        MESSAGE_ERROR("[AgentCommunication][run]: " << ec.message());
    }
}

boost::signals2::signal<void (std::shared_ptr<MsgToEnvironment>)> &AgentCommunication::get_signal_process_msg_to_environment()
{
    return signal_process_msg_to_environment;
}

void AgentCommunication::process_msgs_to_agent(std::shared_ptr<MsgToAgent> msg)
{
    this->write_msg_to_agent(msg);
}
