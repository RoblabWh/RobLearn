#include "gz_environment/environment.h"
#include "gz_environment/status_helper.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


#include <iostream>



int main(int argc, char *argv[])
{
    Environment env;

    env.init(argc, argv);
    env.run();


    return EXIT_SUCCESS;
}
