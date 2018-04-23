#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "gz_environment/gazebo_communication.h"
#include "gz_environment/agent_communication.h"

#include <boost/signals2.hpp>


class Environment
{
private:
    std::unique_ptr<GazeboCommunication> gazebo_communication;
    std::unique_ptr<AgentCommunication> agent_communication;

    void set_signals();
    void synchronize_laserscan(unsigned int steps);

public:
    Environment();
    bool init(int argc, char *argv[]);
    void run();
};

#endif //ENVIRONMENT_H
