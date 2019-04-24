# Roblearn
Deep learning of a mobile robot equipped with a laser scanner and a RGB-D camera to navigated in unknown environments.

![Alt text](images/RoblearnRoboterplattform.png?raw=true "Robot")

Deep Reinforcement Learning has been successfully applied in various computer games. But it is still rarely used in real world applications especially for the navigation and continuous control of real mobile robots. Previous approaches lack of safety and robustness and/or need environmental interventions. In this project, we present our proof of concept to learn robot navigation in an unknown environment for a real robot without a map or planer. The input for the robot is only the fused data from a 2D laser scanner and a RGB-D camera as well as the orientation to the goal. The map of the environment is unknown. The output actions of an Asynchronous Advantage Actor-Critic network (GA3C) are the linear and angular velocities for the robot. The  navigator/controller network is pretrained in a very fast, parallel, self implemented simulation environment to speed up the learning process and then deployed to the real robot. To avoid over fitting we train relative small networks, and we add random Gaussian noise to the input laser data. The sensor data fusion with the RGB-D camera allows the robot to navigate in real environments with real 3D obstacle avoidance and without environmental interventions. To further increase the robustness we use different levels of difficulties of the environment and train 32 of them simultaneously.

# Video
Youtube https://www.youtube.com/watch?v=KyA2uTIQfxw, https://www.youtube.com/watch?v=skpUU6ggbCo

# Installation
The software was developed under Ubuntu 16.04 with tensorflow (GPU and CPU works)

clone the Repo
```
git clone https://github.com/RoblabWh/RobLearn
```

and install dependencies

```
sudo apt-get install libeigen3-dev libboost-python-dev libboost-system-dev python3-dev inkscape python3-pip gnuplot-qt
```

For the 2D simulation do 

```
cd NeuronalNetwork 
bash build.sh
(or bash build_avx.sh if our processor support AVX)
```
For the installation of Cuda 9.0 see also cuda developer + libcudnn -> nvidia

```
sudo dpkg -i libcudnn7_7.3.1.20-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.3.1.20-1+cuda9.0_amd64.deb 
```

```
sudo -H pip3 install tensorflow-gpu keras numpy
```

If you can run example_dqn.py it works!!!

```
python3 example_dqn.py
```

__Hint: Start the programm in a terminal since gnuplot blocked the desktop.__

# Training
```
./_ga3c_clean.sh &&
./_ga3c_train.sh
```
The repository contains also some intermediate step with different architectures and simulation environements (Gazebo, torse). Developers can check the directories. The directory ROS contains the software to run the trained network at a real robot e.g. turtle bot 2.

 # Dependencies
- tensorflow
- keras
- gnuplot-x11 (Visualierung)

![Alt text](images/simulation.png?raw=true "Simulation")

_LEVEL Modus_

The modes control different environments. See worlds for details.

- "test"       --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (easy)      -- Must work
- "diff_forms" --> set_mode(Mode.PAIR_COMBINATION, terminate_at_end=True)(easy)      -- Must work
- "roblab"     --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (medium)    -- Start is hard
- "room"       --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (hard)      -- Success would be very nice
- "four_rooms" --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (very hard) -- Challenger

__Issues:__
- For ubuntu 18.04 change python 3.5 to 3.6 and (python-py35 -> python.py36) in Simulation2d/CMakeLists.txt
- On some laptops sometimes gnuplot crashed (rarly).


# Citation
```bibtex
@article{Surmann:2019,
  title={Deep Reinforcement learning for real autonomous mobile robot navigation in indoor environment},
  author={{Hartmut Surmann, Christian Jestel, Robin Marchel, \\Franziska Musberg, Houssem Elhadj and Mahbube Ardani},
  journal={},
  year={2019},
  publisher={}
}
```
# Credits
Nvidia for GA3C, Google for tensorflow, Keras, Gnuplot, Ubuntu

# Know Problems
could not found the following .. boost_python_py35 
in CMakeList.txt line 19 
find_package(Boost REQUIRED COMPONENTS system python-py35) #python-py36
for Ubuntu 16.04
