# Roblearn

Repository clonen

```
git clone https://github.com/RoblabWh/RobLearn
```

Abhängigkeiten

```
sudo apt-get install libeigen3-dev libboost-python-dev libboost-system-dev gnuplot-x11 python3-dev inkscape python3-pip
```

2D Simulation installieren

```
cd NeuronalNetwork 
bash build.sh
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

