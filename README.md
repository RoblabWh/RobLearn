# Roblearn
Training of a mobile robot equipped with a laser scanner and a RGB-D camera to navigated in unknown environments.

# Installation
The software was developed under Ubuntu 16.04 with tensorflow.

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
For the installation of Cudo 9.0 see also cuda developer + libcudnn -> nvidia

```
sudo dpkg -i libcudnn7_7.3.1.20-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.3.1.20-1+cuda9.0_amd64.deb 
sudo -H pip3 install tensorflow-gpu keras numpy
```

If you can run example_dqn.py it works!!!

```
python3 example_dqn.py
```

__Hint: Start the programm in a terminal since gnuplot blocked the desktop.__

# Training
``
./_ga3c_clean.sh
./_ga3c_train.sh
```

 # Dependencies
- tensorflow
- keras
- gnuplot-x11 (Visualierung)

_LEVEL Modus_
The modes control different environments. See worlds for details

- "test"       --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (easy)      -- Must work
- "diff_forms" --> set_mode(Mode.PAIR_COMBINATION, terminate_at_end=True)(easy)      -- Must work
- "roblab"     --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (medium)    -- Start is hard
- "room"       --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (hard)      -- Success would be very nice
- "four_rooms" --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (very hard) -- Challenger

__Issues:__
- For ubuntu 18.04 change python 3.5 to 3.6 and (python-py35 -> python.py36) in Simulation2d/CMakeLists.txt
- On some laptops sometimes gnuplot crashed (rarly).
