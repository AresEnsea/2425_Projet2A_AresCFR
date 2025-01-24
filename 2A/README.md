# CDF-2025
Github de la Coupe de France de Robotique!

#REGLEMENTS
https://www.coupederobotique.fr/accueil/le-concours/reglement-2025/
https://www.coupederobotique.fr/wp-content/uploads/Eurobot2025_Rules.pdf

#DIAPO 
[https://enseafr-my.sharepoint.com/:p:/g/personal/antoine_lemarignier_ensea_fr/Eb2eWKZeK-NNrEKdino8REcB281sSVFofJ1WQgk0IBzqbg?e=ZdGJZ5](https://enseafr-my.sharepoint.com/:p:/g/personal/antoine_lemarignier_ensea_fr/ESafam9fs7ZCg727WFCmVboB-b12O3MPg7kT_spCKCZYLA?e=d9OOJI)

#CAD
https://cad.onshape.com/documents/e7dd6126290aaac8861c9b96/w/d5c385b6723083ea50ea9e11/e/9022f36f38f961f67dfb7efc?renderMode=0&uiState=66fbe8290d120e0a630e1b35

## Photo Robot :
<img src="https://github.com/user-attachments/assets/f719fcd2-1e79-4abe-89d7-a2176633c4ac" alt="IMG_20250124_191657" width="500"/>


## Liste des participants :
- Lorenzo ROMEO
- Antoine LEMARIGNER
- Mateo GOMES
- Nathan BAINARD
- Ewan ZAHRA-THENAULT
- Khalid ZOUHAIR
- Abderhamane EL FELSOUFI 
- Mohamed EL KOURMISS 
- Sammy GROS
- Matis GARBEZ
- Kenny SAINT FLEUR

## Partenaires :
- [x] Wurth
- [x] Elsys Design
- [x] RS
- [x] Acksys

# Etat du Projet 

## PCB MOTHERBOARD : 
- lien : https://github.com/AresEnsea/2425_Projet2A_AresCFR/edit/main/2A/Hardware/Hardware_%C3%A9lectronique/MainBoard/PCB_test/
- [x] Actuellement en V2
## PCBs auxiliaires :
- [x] steppers https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Stepper
- [x] Moteurs base roulante https://github.com/AresEnsea/2425_Projet2A_AresCFR/new/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Moteur
- [x] Servo moteurs https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Servo

## Méca :
lien modèle 3D : https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_M%C3%A9canique
- [x] Base roulante du robot complète
- [x] Modèle de Pince final (quasi)
- [x] Pancarte
- [ ] Plus rien à faire mécaniquement

## ROS2 embarqué
- lien workspace ros : https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Firmware/ros2_ws
- [x] Liaison UART pi-STM32
- [x] Rosbags moteurs-actionneurs-Lidar
- [x] Interface écran
- [ ] Odométrie
- lien workspace lidar : https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Firmware/ydlidar_ros2_ws
- [x] Scan fonctionnel
- [ ] Tout mettre dans le workspace principal

## STM32
lien projet STM32 : https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Firmware/STM32_moteurs
- [x] V1 code stm32 moteurs réagissant à la pi
- [ ] V2 sur le microprocesseur du PCB

## Vision
lien projet Vision : https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Vision
- [x] Calibration caméra
- [x] Détection Aruco de la table
- [x] Suivi du mouvement d'un Aruco
- [ ] Transformer les informations de la caméra en carte pour le ROS

## Réseau
- [x] Apprendre à utiliser un routeur
- [x] Ping à travers 2 routeurs
- [ ] Avoir un réseau transportable à la Coupe
- [ ] Connecter la pi et la jetson à un même réseau wifi pour communiquer
   
# Répartition par tâche :

## 1-ROS2/STM32 :
- Antoine
- Mateo
- Sammy

Le compte rendu de notre équipe sur le software embarqué se trouve dans le ReadMe.md du dossier Firmware :
https://github.com/AresEnsea/2425_Projet2A_AresCFR/blob/d943063ebb5853ef2ae5634fa93b110d4156a13e/2A/Firmware/README.md

## 2-Traitement Image :
- Khalid
- Mohamed
- Abderahmane

Le compte rendu de notre équipe sur le traitement d'image est présent ici :
https://github.com/AresEnsea/2425_Projet2A_AresCFR/blob/main/2A/Vision/README.md

## 3-PCB :
- Kenny
- Ewan

Le compte rendu de notre équipe sur le PCB est présent ici :
https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_%C3%A9lectronique

## 4-Mécanique :
- Lorenzo
- Matis
- Bilal
- Sammy 

## 5-Réseaux :
- Nathan

## 6-Planification :
- Mateo
- Antoine 

