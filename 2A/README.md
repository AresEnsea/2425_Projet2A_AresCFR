# CDF-2025
Github de la Coupe de France de Robotique!

# REGLEMENTS
https://www.coupederobotique.fr/accueil/le-concours/reglement-2025/
https://www.coupederobotique.fr/wp-content/uploads/Eurobot2025_Rules.pdf

# DIAPO 
[https://enseafr-my.sharepoint.com/:p:/g/personal/antoine_lemarignier_ensea_fr/Eb2eWKZeK-NNrEKdino8REcB281sSVFofJ1WQgk0IBzqbg?e=ZdGJZ5](https://enseafr-my.sharepoint.com/:p:/g/personal/antoine_lemarignier_ensea_fr/ESafam9fs7ZCg727WFCmVboB-b12O3MPg7kT_spCKCZYLA?e=d9OOJI)

#CAD
https://cad.onshape.com/documents/e7dd6126290aaac8861c9b96/w/d5c385b6723083ea50ea9e11/e/9022f36f38f961f67dfb7efc?renderMode=0&uiState=66fbe8290d120e0a630e1b35


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


## PCB MOTHERBOARD : 
- lien : https://github.com/AresEnsea/2425_Projet2A_AresCFR/edit/main/2A/Hardware/Hardware_%C3%A9lectronique/MainBoard/PCB_test/
- [x] Actuellement en V2
## PCBs auxiliaires :
- [x] steppers https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Stepper
- [x] Moteurs base roulante https://github.com/AresEnsea/2425_Projet2A_AresCFR/new/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Moteur
- [x] Servo moteurs https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_%C3%A9lectronique/Auxiliairy_board/PCB_Servo

## Soft auxiliaires :
- [ ] Lib (?)
- [ ] XL320/430

## Vision (Python / OpenCV) :
- [ ] Nvidia Jetson

## Communication Wifi

## Mécanique

- Moteurs Maxon
- Base roulante
- Dèploiement de banderole
- Pince élévatrice

## Partenaires :
- [x] Wurth
- [x] Elsys Design
- [x] RS
- [x] Acksys


## Règlement (experts) :
- [ ] Sammy

 ## 0-Avancées génrales :
 - établissement d'une communication ssh acec la py 
 - Installation de ROS2 sur la py
 - Connection en huart entre la py et la stm32
 - possibilité d'un envoie de message depuis la py vers la stm32 pour controler des moteurs et définir les valeurs de vitesses souhaités
 - Possibilité de controler la base roulante avec le clavier
 - ROSBAG ( possibilité de rejouer une séquence faite avec le robot )
 - Lidar ( pour l'instant, il ya bien une évoltion dans le bon sens de la distance évaluée par le lidar lorsqu'un objet se déplace mais cette distance n'est pas encore très précise ) 
 - V2 pour le PCB Moteur et la MainBoard.
 - PCB auxiliaire pour les stepper et les servo.
   
## Répartition par tâche :

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

  Le compte rendu de notre équipe sur la mécanique est présent ici :
  [2A/Hardware/Hardware_Mécanique](https://github.com/AresEnsea/2425_Projet2A_AresCFR/tree/main/2A/Hardware/Hardware_M%C3%A9canique)

## 5-Réseaux :
- Nathan

## 6-Planification :
- Mateo
- Antoine 

