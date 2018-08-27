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
    // Allocate the buffer
    this->buffer_read_msg = new char [STD_MEMORY_ALLOCATED_BUFFER_READ_MSG];
    this->buffer_read_msg_length = new char [STD_MEMORY_ALLOCATED_BUFFER_READ_LENGTH_MSG];
    this->buffer_write_msg = new char [STD_MEMORY_ALLOCATED_BUFFER_WRITE_MSG];

    // Listen to an tcp commincation from the python agent.
    acceptor.listen();
    acceptor.async_accept(socket,boost::bind(&AgentCommunication::handler_accept,this, boost::asio::placeholders::error));

    MESSAGE_INFO("Wait for agent connection on port: " << port << ".");

    is_terminated = false;
}

AgentCommunication::~AgentCommunication()
{
    // delete the buffer
    delete this->buffer_read_msg;
    delete this->buffer_read_msg_length;
    delete this->buffer_write_msg;

    // Close all tcp communication
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
        MESSAGE_INFO("Connection established!");
        // close the acceptor for only one connection and wait for the incommung messages.
        this->acceptor.close();
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

        // read the length of the next incomming message
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

        // Change the byte order when the system is little endian
#if __BYTE_ORDER == __ORDER_LITTLE_ENDIAN__
        msg_length = bswap_32(msg_length);
#endif
        // Read the content of the message
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

    // Change the byteorder for little endian system
#if __BYTE_ORDER == __ORDER_LITTLE_ENDIAN__
    msg_length = bswap_32(msg_length);
#endif


    memcpy(buffer_write_msg, &msg_length, 4);

    msg->SerializeToArray(buffer_write_msg + 4, msg->ByteSize());

    // Write the protobuffer message to the agent
    boost::asio::async_write(socket, boost::asio::buffer(buffer_write_msg, total_length), boost::bind(&AgentCommunication::handler_write_msg_to_agent, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::read_msg_to_environment_length()
{
    // Activate the handler for reading the message length
    boost::asio::async_read(socket, boost::asio::buffer(buffer_read_msg_length,4), boost::asio::transfer_exactly(4),boost::bind(&AgentCommunication::handler_read_msg_to_environment_length, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::read_msg_to_environment(unsigned int msg_length)
{
    // Activate the handler for reading the message content
    boost::asio::async_read(socket, boost::asio::buffer(buffer_read_msg, msg_length), boost::asio::transfer_exactly(msg_length),boost::bind(&AgentCommunication::handler_read_msg_to_environment, this, boost::asio::placeholders::error ,boost::asio::placeholders::bytes_transferred));
}

void AgentCommunication::run()
{
    // Loop until the communication is termited.
    while (!is_terminated)
    {
        try {
            // Run the io_service for the communication handling
            io_service.run();
        } catch (boost::system::error_code ec){
            // Try to reconnect when the communication is abort or eof
            if (boost::asio::error::make_error_code(boost::asio::error::eof) == ec || boost::asio::error::make_error_code(boost::asio::error::connection_reset) == ec) {
                MESSAGE_WARN("[AgentCommunication][run]: " << ec.message());

                this->reconnecting();
            }
            else
            {
                MESSAGE_ERROR("[AgentCommunication][run]: " << ec.message() << ". Terminated the program.");
                is_terminated = true;
            }
        }
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

bool AgentCommunication::terminated()
{
    return is_terminated;
}

void AgentCommunication::reconnecting()
{
    MESSAGE_INFO("Reset connection.");

    io_service.reset();

    this->acceptor.close();
    this->socket.close();

    this->acceptor = boost::asio::ip::tcp::acceptor(this->io_service, this->endpoint);
    this->socket = boost::asio::ip::tcp::socket(this->io_service);

    acceptor.listen();
    acceptor.async_accept(socket,boost::bind(&AgentCommunication::handler_accept,this, boost::asio::placeholders::error));

    MESSAGE_INFO("Ready for reconnecting.");
}
