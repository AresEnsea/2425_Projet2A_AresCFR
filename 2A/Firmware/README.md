Ici le dossier Firmware du Github de la Coupe de France 2025 d'Ares, il regroupe les fichiers C,C++ et Python qui seront utilisés pour programmer le robot principal.

# Contenu
- CFR_G474RET6 : le projet STM32CubeIDE que nous implémenteront ultérieurement dans le microprocesseur (G474RET6) du pcb, pour le moment il contient simplement le pinout
- STM32_Pi_Connexion : contient le projet qui STM32CubeIDE qui permet la connexion UART entre une STM32CubeIDE L476RG (avant de pouvoir utiliser le PCB) et une raspberry pi4
- STM32_moteurs : contient le projet STM32CubeIDE qui délivre deux PWM pour faire tourner des moteurs

- ros2_ws : le workspace ROS actuellement sur la pi4 (booté avec Ubuntu 22.04 et utilisant ROS2 
  >- serial_node.py : on envoie une succession de chiffre (messages de 8 chiffres) en UART vers la STM
  >- serial_node_2.py : on envoie le message en fonction de l'input fait sur le clavier ce qui permet de controller par exemple la base roulante à distance
  >- serial_node_3.py : semblable à la node précédente mais on enrgistre en plus le message envoyé à la stm dans un rosbag pour pouvoir le rejouer ensuite (on créé notre propre topic "keyboard_commands")
  >- serial_node_4.py : on a une node "attentive" aux messages publiés sur le topic "keyboard_commands", qui attends les messages et les envoie dans l'UART.
  >- test_uart_lidar.py : on publie les distances (normalement) mesurées par le Lidar (reliés directement en UART à la pi), mais les valeurs varient beaucoup assez vite, donc à corriger ( faire des moyeenes ou un seuil de décision spécial).

- ydlidar_ros2_ws : le workspace du lidar qui nous sert de test pour l'instant mais qui sera rajouté dans notre workspace principal
   >- ydlidar_launch_view.py : avec un logiciel de visualisation 3D, nous voyons l'ensemble des points définis par le lidar.
   >- ydlidar_launch.py : renvoie les distances prises en considération par le lidar.
<image src="https://github.com/user-attachments/assets/174b4c74-a171-4f48-ac5a-31af73da478c" alt="image" width=300/>

  
- ecran_code_2425 : projet permettant d'afficher sur un écran lcd le plateau de jeu de la coupe de France. A l'avenir, nos aurons chaque rosbag associé à chaque position de départ.
<image src="https://github.com/user-attachments/assets/786938ef-1c37-4f8c-b087-20cfafe1b13b" alt="image" width=300/>



## Protocole de communication :
Message de taille N, à augmenter selon les besoins 
"M . . . . . . . .", 9 caractères pour les moteurs : 4 pour le droit et 4 pour le gauche
(à faire : "A . . . ", [0,1,2] pour l'actionneur à la position correspondante, respectivement pour 0,90 et 180 degrés)

## Liste des topics 
-"keyboard_commands" envoyés par les nodes "serial_node_X" pour communiquer en UART avec la STM32, de la forme décrite ci-dessus.
-"scan" topic propre au lidar qui envoie les ranges (float) et l'intensité (float)  du laser pour chaque angle.
-"stop" le topic booléen que l'on tournera a 1 si le robot doit s'arrêter, il sera alors communiqué au reste des nodes du robots (moteurs, actionneurs).


# ROS Terms
- Rosbag : fichier qui enregistre des données des topics ROS pour les rejouer ensuite.
--> un rosbag peut par exemple enregistrer les données d'un topic "lidar" et d'un topic "wheels" pour rejouer les données (extension .deb3)
- Topic : message de types différents (int, bool, char, ...) qui s'échangent entre les nodes (node pubisher et node subscriber)
--> pour un Lidar, les données peuvent être envoyées avec un topic "lidar_data" avec un en-tête qui donne les paramètres puis les données de tout type (voir le package lidar-driver).
- Node : processus (fichier en python ou en C++ ici) qui effectue une action, un projet comprend généralement de nombreux nœuds.
--> dans notre projet par exemple les fichiers "serial_node_X.py" envoie de manière différente des données via uart à la pi

# ROS workspace objectif
![image](https://github.com/user-attachments/assets/d4c207fe-e565-4de2-9e26-0dd949e4befa)

# Fait 
- Communication Pi-STM32 : notre code en C avec les nodes ROS nous permettent de controler plusieurs PWM grâce à un message en UART envoyés à intervalles réguliers (ainsi que les GPIOs)
- Rosbag : nous pouvons enrgistrer aisément des bags avec les messages envoyés à l'UART même si nous semblons rencontrer certaines difficultés à les rejouer avec le bon timing, nous y travaillons.
- Lidar : une node ROS nous permet de lire les données du Lidar.

# A venir :
- Odométrie : dès que nous aurons un PCB opérationel nous nous empresserons de commencer l'odométrie 
- Rosbag : nous aimerions également enrgistrer au plus vite des bags nous permettant de rejouer les messages envoyés aux moteurs et aux servos pour commencer à faire des tenatives de constructions de "gradins".
- Lidar : Il manque l'implémentation d'une node d'arrêt en fonction des données du Lidar (avec un topic "stop").
