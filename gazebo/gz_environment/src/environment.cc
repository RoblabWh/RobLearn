#include "gz_environment/environment.h"
#include "boost/signals2.hpp"

const static unsigned int STD_PORT_AGENT_CONNECTION_ADDITION = 10000;

Environment::Environment()
{

}

bool Environment::init(int argc, char *argv[])
{
    gazebo_communication = std::unique_ptr<GazeboCommunication>(new GazeboCommunication());
    gazebo_communication->init(argc, argv);

    agent_communication = std::unique_ptr<AgentCommunication>(new AgentCommunication(gazebo_communication->get_port() + STD_PORT_AGENT_CONNECTION_ADDITION));

    this->set_signals();
}

void Environment::run()
{
    MESSAGE_INFO("Starting laserscan synchronisation!");
    this->synchronize_laserscan(10);

    MESSAGE_INFO("Laserscan synchronisation done!");

    agent_communication->run();
}

void Environment::set_signals()
{
    this->agent_communication->get_signal_process_msg_to_environment().connect(boost::bind(boost::mem_fn(&GazeboCommunication::process_msg_to_environment), boost::ref(this->gazebo_communication), _1));
    this->gazebo_communication->get_signal_process_msg_to_agent().connect(boost::bind(boost::mem_fn(&AgentCommunication::process_msgs_to_agent), boost::ref(this->agent_communication), _1));
}

void Environment::synchronize_laserscan(unsigned int steps)
{
    this->gazebo_communication->send_world_control_pause();

    // wait 0.5 seconds to pause the simulation and ensure that no laserscan is published anymore;
    usleep(5000000);

    // set the state laserscan state to syncing
    this->gazebo_communication->set_state_laser(GazeboCommunication::STATE_LASER::SYNCING);

    while(this->gazebo_communication->get_state_laser() == GazeboCommunication::STATE_LASER::SYNCING)
    {
        gazebo_communication->send_world_control_steps(steps);

        // wait for a while of the laserscan message
        usleep(10000);
    }
}
