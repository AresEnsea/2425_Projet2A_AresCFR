Ici le dossier Firmware du Github de la Coupe de France 2025 d'Ares, il regroupe les fichiers C,C++ et Python qui seront utilisés pour programmer le robot principal.

# Algorigramme Robot 
<img width="725" alt="image" src="https://github.com/user-attachments/assets/e0e04d4b-9f4d-4145-87f1-bcad0d0d5a1c" />

Au démarrage du robot, regroupé sous l'action Init, plusieurs fonctions se lancent : poser la bannière, démarrage du package LiDAR. Ensuite l'action du robot reste simple : il suit le chemin indiqué par la jetson, exécute l'action de poser ou de prendre les canettes/planches et recommence. La fin de l'opération cannettes/planches sera vérifiée grâce à des switch. 

# Contenu

- ros2_ws : le workspace ROS actuellement sur la pi4 (booté avec Ubuntu 22.04 et utilisant ROS2 
  >- serial_node.py : on envoie une succession de chiffre (messages de 8 chiffres) en UART vers la STM
  >- serial_node_2.py : on envoie le message en fonction de l'input fait sur le clavier ce qui permet de controller par exemple la base roulante à distance
  >- serial_node_3.py : semblable à la node précédente mais on enrgistre en plus le message envoyé à la stm dans un rosbag pour pouvoir le rejouer ensuite (on créé notre propre topic "keyboard_commands")
  >- serial_node_4.py : on a une node "attentive" aux messages publiés sur le topic "keyboard_commands", qui attends les messages et les envoie dans l'UART.
  >- test_uart_lidar.py : on publie les distances (normalement) mesurées par le Lidar (reliés directement en UART à la pi), mais les valeurs varient beaucoup assez vite, donc à corriger ( faire des moyeenes ou un seuil de décision spécial).
  >- tirette_node.py : lié au topic "tirette_value" qui donne le départ pour lancer un bag dans la node robot_control et de lancer le robot.
  >- screen_reader : reçoit de l'uart entre l'écran et la pi le file que l'on souhaite lancer puis le publie sur le topic "file_stm_choisi"
  >- robot_control : (plusieurs) node(s) qui améliore les serial node 
  >- bagfile_stm32.py : node attentive au topic "file_stm_choisi", qui le lance en réaction (le bag permettant 
  >- launch_robot : node qui regroupe toutes les actions du robot ci dessus (lidar, écran, connexion série)
  >- ros2_startup.sh : script shell qui lance le launch au branchement de la pi (ne nécessite pas de wifi) 

- ydlidar_ros2_ws : le workspace du lidar qui nous sert de test pour l'instant mais qui sera rajouté dans notre workspace principal
   >- ydlidar_launch_view.py : avec un logiciel de visualisation 3D, nous voyons l'ensemble des points définis par le lidar.
   >- ydlidar_launch.py : renvoie les distances prises en considération par le lidar.
<image src="https://github.com/user-attachments/assets/174b4c74-a171-4f48-ac5a-31af73da478c" alt="image" width=300/>

-tirette : la tirette est associée à un gpio input et la pi en fait un topic binaire permettant de lancer un bag en début de partie <br/>
<img src="https://github.com/user-attachments/assets/65d34960-c1c1-4513-9c7e-b8fa18f0437f" width="300" />

- code stm : 
  - cfr_mot2 : code pour la "carte mère", la carte redistribue le message reçue de la py dans les autres stm.

  - servo_ares_board : code qui permet de contrôler les servos moteurs.

  - stepper_ares : code qui récupère le message en uart et qui contrôle les stepper.
  
  - ecran_code_2425 : projet permettant d'afficher sur un écran lcd le plateau de jeu de la coupe de France. 
<image src="https://github.com/user-attachments/assets/786938ef-1c37-4f8c-b087-20cfafe1b13b" alt="image" width=300/>




## Protocole de communication :
Message de taille N, à augmenter selon les besoins 
Pour le moment le protocole est le suivant : " dir_mot_D | speed_mot_D | dir_mot_G | speed_mot_G  | servo | stepper"
Avec le nombres de bytes accordés à chaque variables : "1 | 4 | 1 | 4 | 12 | 4" 

https://github.com/user-attachments/assets/22d95b05-f980-4ab3-9817-96d0321312c0

Soit 26 caractères au total, à savoir que le premier bit de chaque moteur donne le sens les bytes derrière la valeur de vitesse (réinterprétée par la stm), les servo varient tous entre 0 et 1 et les stepper entre 0, 1 et 2 pour pouvoir les faire monter, descendre et laisser immobiles.

## Liste des topics 
-"keyboard_commands" envoyés par les nodes "serial_node_X" pour communiquer en UART avec la STM32, de la forme décrite ci-dessus. <br/>
-"scan" topic propre au lidar qui envoie les ranges (float) et l'intensité (float)  du laser pour chaque angle.<br/>
-"stop_moteur" le topic booléen que l'on tournera a 1 si le robot doit s'arrêter, il sera alors communiqué au reste des nodes du robots (moteurs, actionneurs).<br/>
-"bagfile_stm32" le topic dans lequel la node 
-"tirette_value" binaire qui permet ou non de lancer le bag en fonction de l'état de la tirette 
-"file_stm_choisi" en fonction du bouton cliqué sur la stm32 disco (écran)

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
- Communication Pi-STM32 : notre code en C avec les nodes ROS nous permettent de controler plusieurs PWM grâce à un message en UART envoyés à intervalles réguliers (ainsi que les GPIOs).
- Rosbag : nous pouvons enrgistrer aisément des bags avec les messages envoyés à l'UART même si nous semblons rencontrer certaines difficultés à les rejouer avec le bon timing, nous y travaillons.
- Lidar : une node ROS nous permet de lire les données du Lidar.
- Ecran : l'écran nous permet de selectionner un bag.
- Commande pinces : on peut aisément controler les servo et les stepper depuis la pi.
- Tirette : la tirette permet de donner le départ.

# A venir :
- Odométrie : dès que nous aurons un PCB opérationel nous nous empresserons de commencer l'odométrie 
