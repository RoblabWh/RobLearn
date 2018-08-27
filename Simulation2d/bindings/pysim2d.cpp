#include "simulation2d/simulation2d.h"

#include <boost/python.hpp>

/**
 * Boost python bindings to get the values from the 2dsimulation.
 */
using namespace boost::python;

BOOST_PYTHON_MODULE(pysim2d)
{
    class_<Simulation2D>("pysim2d")
        .def("init", &Simulation2D::init)
        .def("step", &Simulation2D::step)
        .def("visualize", &Simulation2D::visualize)
        .def("set_robot_pose", &Simulation2D::set_robot_pose)
        .def("get_robot_pose_x", &Simulation2D::get_robot_pose_x)
        .def("get_robot_pose_y", &Simulation2D::get_robot_pose_y)
        .def("get_robot_pose_orientation", &Simulation2D::get_robot_pose_orientation)
        .def("observation_size", &Simulation2D::observation_size)
        .def("observation_at", &Simulation2D::observation_at)
        .def("observation_min_clustered_at", &Simulation2D::observation_min_clustered_at)
        .def("observation_min_clustered_size", &Simulation2D::observation_min_clustered_size)
        .def("done", &Simulation2D::done);
}
