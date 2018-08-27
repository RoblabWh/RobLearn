#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "gz_environment/gazebo_communication.h"
#include "gz_environment/agent_communication.h"

#include <boost/signals2.hpp>

/**
 * @brief The Environment class This calls provide the communication between the gazebo server and the python agent.
 */
class Environment
{
private:
    /**
     * @brief gazebo_communication Communication to the gazebo sever
     */
    std::unique_ptr<GazeboCommunication> gazebo_communication;
    /**
     * @brief agent_communication Communication to the python agent
     */
    std::unique_ptr<AgentCommunication> agent_communication;

    /**
     * @brief set_signals Bind the signal to the communication classes
     */
    void set_signals();

    /**
     * @brief synchronize_laserscan Make small gazebo simulation step until the laserscan message is recieved.
     * @param steps Simulation step.
     */
    void synchronize_laserscan(unsigned int steps);

public:
    Environment();

    /**
     * @brief init Initialize the class.
     * @param argc Argument counter.
     * @param argv Arguments.
     * @return
     */
    bool init(int argc, char *argv[]);

    /**
     * @brief run Start the communication.
     */
    void run();
};

#endif //ENVIRONMENT_H
