#include "simulation2d/simulation2d.h"
#include "simulation2d/robot.h"
#include "simulation2d/data_container.h"
#include "simulation2d/line.h"


#include <iostream>
#include <stdlib.h>
#include <chrono>




int main(int argc, char *argv[])
{
    Simulation2D simulation;

    simulation.init("");

    auto start = std::chrono::steady_clock::now();


    simulation.set_robot_pose(1,0,0);

    for (int i = 0; i < 1; ++i) {


        for (int j = 0; j < 40; ++j) {
            simulation.step(0.0,0.0);
        }
        //std::cout << i << std::endl;
        simulation.visualize(0,0,1);

        //Eigen::Affine2f pose = simulation.get_robot_pose();
        //std::cout << pose.matrix() << std::endl;
    }

    std::cout << std::flush;

    auto end = std::chrono::steady_clock::now();

    auto diff = end - start;

    std::cout << "time: " << std::chrono::duration <double, std::milli> (diff).count()<< std::endl;





    return EXIT_SUCCESS;
}

