cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(simulation2d)

add_compile_options(-std=c++11)

option(WITH_AVX "use avx" OFF)

if (WITH_AVX)
  add_compile_options(-mavx)
  MESSAGE(STATUS "WITH_AVX")
  add_definitions(-DUSE_AVX)
endif()

find_package(Eigen3 REQUIRED)

find_package(PythonInterp 3.5 REQUIRED)
find_package(PythonLibs 3.5 REQUIRED)
find_package(NumPy REQUIRED)

find_package(Boost REQUIRED COMPONENTS system python-py35)


#find_package(SDL2 REQUIRED)

#include_directories(${SDL2_INCLUDE_DIRS})
#set(SDL2_LIBRARIES "-L${SDL2_LIBDIR} -lSDL2")


aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST )
include_directories(${PROJECT_SOURCE_DIR}/include )
include_directories(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

#add_executable(${PROJECT_NAME} ${SRC_LIST})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

PYTHON_ADD_MODULE(pysim2d SHARED bindings/pysim2d.cpp ${SRC_LIST})
