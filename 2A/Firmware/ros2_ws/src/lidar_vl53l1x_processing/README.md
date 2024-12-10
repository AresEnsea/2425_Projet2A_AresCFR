# lidar_vl53l1x_processing pkg

Ce package ROS2 humble permet de récupérer les informations fournies par le système de détection du robot (16 capteurs TOF, STM32 VL53L1X). Il s'agit d'un publisher qui envoie une information en fonction de la distance minimale récupérée. 

### Pour l'utiliser : 

Ajouter le dossier 'lidar_vl53l1x_processing' dans le dossier /src de votre workspace ROS2.

Il ne faut pas oublier de source votre environnement ROS2 :
```
source /opt/ros/humble/setup.bash
```

À la racine de votre workspace exécuter les commandes suivantes :

```
colcon build --packages-select lidar_vl53l1x_processing
```
```
source install/setup.bash
```
Cette commande affichera la distance minimale ainsi que l'information publiée par la node :
```
ros2 run lidar_vl53l1x_processing lidar_ensea
```

### Troubleshooting

Vous aurez probablement besoin d'installer la librairie "pyserial" nécessaire pour la communication série avec le lidar :
```
pip install pyserial
```
