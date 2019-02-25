# Roblearn

Repository clonen

```
git clone https://github.com/RoblabWh/RobLearn
```

Abhängigkeiten

```
sudo apt-get install libeigen3-dev libboost-python-dev libboost-system-dev python3-dev inkscape python3-pip gnuplot-qt
```

2D Simulation installieren

```
cd NeuronalNetwork 
bash build.sh
```

Cuda 9.0  installieren (siehe -> cuda developer) + libcudnn (-> nvidia)

```
sudo dpkg -i libcudnn7_7.3.1.20-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.3.1.20-1+cuda9.0_amd64.deb 
sudo -H pip3 install tensorflow-gpu keras numpy
```

example_dqn.py ausführen

```
python3 example_dqn.py
```

__Hinweis: Example im Terminal starten, da die Gnuplot-Visualierung den Desktop blockiert__

Abhängikeiten
- tensorflow
- keras
- gnuplot-x11 (Visualierung)

Probleme:
- Manchmal klappt die Gnuplot-Visualierung nicht.

_LEVE Modus_
- "test"       --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (easy)      -- Must work
- "diff_forms" --> set_mode(Mode.PAIR_COMBINATION, terminate_at_end=True)(easy)      -- Must work
- "roblab"     --> set_mode(Mode.ALL_COMBINATION, terminate_at_end=True) (medium)    -- Start is hard
- "room"       --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (hard)      -- Success would be very nice
- "four_rooms" --> set_mode(Mode.ALL_RANDOM, terminate_at_end=False)     (very hard) -- Challenger


__Issues:
For ubuntu 18.04 change python 3.5 to 3.6 and (python-py35 -> python.py36) in Simulation2d/CmakeLists.txt
