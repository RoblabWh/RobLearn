# Simulation von Roboter-Navigation mit Morse & EduMorse

Ein elementarer Bestandteil des “RobLearn” Projekt ist die Simulation der Roboter Plattform und ihrer zugehörigen Sensoren, ihrer Umgebung als ein Labyrinth und autonomer Navigation des Roboters in dieser Umgebung. Besonders wichtig ist  die genaue Lokalisierung der Roboter (Position und Orientierung des Roboters zu ermitteln oder zu setzen). Außerdem sollte die Position des Zieles bzw. der Pfad, der die Roboter Plattform zu diesem Ziel hinterlässt simuliert werden. 


Da zusätzliche Sensoren wie ein Laserscanner und eine RGB-D in diesem Projekt Informationen über die Umgebung des mobilen Roboters liefern, ist eine ständige Kommunikation zwischen das neuronale Netz und die Simulation, zur Ermittlung der Sensor Daten bzw. Übertragen von aktuellen Werte notwendig.


Um den zugedachten Zweck zu erreichen, wurde es die Simulationsmöglichkeiten mit diversen 2D- bzw. 3D- Robotik Simulatoren geprüft. Gazebo, Morse Simulator von OpenRobot und darauf basierte EduMorse Framework waren die Kandidaten, die am Ende genau analysiert und kritisiert werden sollten. Aus diesem Grund wurde es ein einfaches Labyrinth als Umgebung und ein Model von TurtleBot als Roboter Plattform moderiert und eine Implementierung von Bug Algorithmus als Navigation Skript mit Morse und EduMorse entwickelt, damit die Möglichkeiten des Simulators kritisieren zu können. 


Das Endergebnis ist hier in “Morse” Ordner zu finden. Der “Bug” Ordner beinhaltet die Navigation System und die Umgebung. Das erstelltes Model von TurtleBot ist im “TurtleBot” zu finden.


Das Model von TurtleBot wurde nicht in Bug Projekt integriert, da im Mitte des Projekt um eine 2D Simulator entschieden wurde und es wurde beschlossen keine weitere Zeit auf 3D Simulatoren zu investieren.
