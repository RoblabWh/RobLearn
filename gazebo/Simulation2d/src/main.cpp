#include "simulation2d/simulation2d.h"
#include "simulation2d/robot.h"
#include "simulation2d/data_container.h"
#include "simulation2d/line.h"

#include <SDL2/SDL.h>

#include <iostream>
#include <stdlib.h>
#include <chrono>



int main(int argc, char *argv[])
{
    //SDL_Init(SDL_INIT_VIDEO);
    Simulation2D simulation;

    simulation.test();
    simulation.init();


    auto start = std::chrono::steady_clock::now();


    simulation.set_robot_pose(0,0,M_PI_4);

    for (int i = 0; i < 2500; ++i) {


        for (int j = 0; j < 40; ++j) {
            simulation.step(0.1,0.05);
        }
        simulation.visualize();


        //Eigen::Affine2f pose = simulation.get_robot_pose();
        //std::cout << pose.matrix() << std::endl;
    }

    std::cout << std::flush;




    auto end = std::chrono::steady_clock::now();

    auto diff = end - start;

    std::cout << "time: " << std::chrono::duration <double, std::milli> (diff).count()<< std::endl;





    return EXIT_SUCCESS;
}
