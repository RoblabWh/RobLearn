# Simulation von Roboter-Navigation mit Morse & EduMorse

Ein elementarer Bestandteil des “RobLearn” Projekt ist die Simulation der Roboter Plattform und ihrer zugehörigen Sensoren, ihrer Umgebung als ein Labyrinth und autonomer Navigation des Roboters in dieser Umgebung. 

Besonders wichtig ist  die genaue Lokalisierung der Roboter (Position und Orientierung des Roboters zu ermitteln oder zu setzen). Außerdem sollte die Position des Zieles bzw. der Pfad, der die Roboter Plattform zu diesem Ziel hinterlässt simuliert werden. 

Da zusätzliche Sensoren wie ein Laserscanner und eine RGB-D in diesem Projekt Informationen über die Umgebung des mobilen Roboters liefern, ist eine ständige Kommunikation zwischen das neuronale Netz und die Simulation, zur Ermittlung der Sensor Daten bzw. Übertragen von aktuellen Werte notwendig.

Um den zugedachten Zweck zu erreichen, wurde es die Simulationsmöglichkeiten mit diversen 2D- bzw. 3D- Robotik Simulatoren geprüft. Gazebo, Morse Simulator von OpenRobot und darauf basierte EduMorse Framework waren die Kandidaten, die am Ende genau analysiert und kritisiert werden sollten. 

Aus diesem Grund wurde es ein einfaches Labyrinth als Umgebung und ein Model von TurtleBot als Roboter Plattform moderiert und eine Implementierung von Bug Algorithmus als Navigation Skript mit Morse und EduMorse entwickelt, damit die Möglichkeiten des Simulators kritisieren zu können. 


Das Endergebnis ist hier in “Morse” Ordner zu finden. Der “Bug” Ordner beinhaltet die Navigation System und die Umgebung. Das erstelltes Model von TurtleBot ist im “TurtleBot” zu finden.


Das Model von TurtleBot wurde nicht in Bug Projekt integriert, da im Mitte des Projekt um eine 2D Simulator entschieden wurde und es wurde beschlossen keine weitere Zeit auf 3D Simulatoren zu investieren.

# installation von Morse & EduMorse

Benötigte Hardware:

```
 Mindestens Intel i5 + 4GB RAM und die Graphik-Karte muss unbedingt GLSL Shading unterstützen.
```
Benötigte Betriebsysteme:

```
 Linux (x86, x86_64) und Apple MacOSX
```

3D Simulation installieren (Ohne Python Binding)

```
$ sudo apt-get install morse-simulator
```
3D Simulation installieren (mit Python Binding)

```
$ sudo apt-get install python3-morse-simulator
```

EduMorse installieren

Dependencies:

```

    -cmake
    -cmake-data
    -pkg-config
    -g++
    -scons
    -openjdk-8-jre-headless
    -openjdk-8-jdk
    -blender
    -git
    -python3
    -python3-dev
    -python3-numpy
    -python3-pip
    -pytoml

```
Edumorse Repository herunterladen:

```
$ git clone https://github.com/lab-robotics-unipv/eduMorse.git
```
Dann mit Hilfe von installer.sh wie folgt installieren:

```
$ chmod +x installer.sh
$ ./installer.sh
```
#Bug und turtlebot_sim Projekte ausführen
 
  1. zum Ordner Bug oder TurtleBot navigieren.
  2. Simulator breitstellen: $ morse run projektname
  3. Script ausführen: $ python musterscript.py

Für Bug Projekt:
```
$ morse run navigation
$ python robot_navigation.py
```

Für Turtlebot Projekt:
```
$ morse run turtlebot_sim
$ python /path zum Projekt/scripts/turtlebot_sim_client.py
```

#Konfiguration des Bug Projektes
Damit man das Projekt mit diversen Roboter, sensoren, aktuatoren und umgebungen testen kann, wurde es einige allgemeine toml Konfigurationsdateien vorgesehen:

1. simulation.toml : Integration von Environment, Robot und Kamera in der Simulation
2. environment_x_y.toml: Umgebungen und Labyrinthe die eingestellt werden können
3. atrv_with_ir.tml: Der Auswahl und Einstellung von Roboter plattform (z.B. atrv) und dazugehörige Sensoren
4. navigation/crash_generator/configuration.toml: Dien Einstellung der Eigenschaften des Algorithmus . z.B. mit welchen Geschwindigkeiten soll der Roboter die Wände folgen usw.

 

