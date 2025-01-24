# CDF-2025
Github de la Coupe de France de Robotique!

#REGLEMENTS
https://www.coupederobotique.fr/accueil/le-concours/reglement-2025/
https://www.coupederobotique.fr/wp-content/uploads/Eurobot2025_Rules.pdf

#DIAPO 
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

## PCB MOTHERBOARD :[https://github.com]
- [ ] alimentation + connecteurs universels :
  - [ ] régulateurs de tension
  - [ ] connecteurs universels sur toute la bordure de la carte (en mettre un maximum)
- [ ] Interface Homme Machine
- [ ] Rapberry Pi 5 (Possibilité d'une Pi 4 pour les bibliothèques?)
- [ ] STM32 a clarifier (photo illisible)
- [ ] Quelque chose d'illisible (surement Wifi?)

Lien reu 1 :[https://enseafr-my.sharepoint.com/personal/ousmane_thiongane_ensea_fr/_layouts/15/stream.aspx?id=%2Fpersonal%2Fousmane%5Fthiongane%5Fensea%5Ffr%2FDocuments%2FRecord%5FR%C3%A9union%5FCFR%2Emp4&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2Ec861fb71%2D2026%2D41c4%2D8537%2Dba26ca0c9958

## PCBs auxiliaires :
- [ ] steppers
- [ ] ESC
- [ ] autes capteurs (...)

## Soft auxiliaires :
- [ ] Lib (?)
- [ ] XL320/430

## Vision (Python / OpenCV) :
- [ ] Nvidia Jetson

## Communication Wifi

## Mécanique

## Partenaires :
- [x] Wurth
- [x] Elsys Design
- [x] RS
- [x] Acksys


## Règlement (experts) :
- [ ] Sammy

 ## 0-Avancées génrales :
 -établissement d'une communication ssh acec la py 
 -Installation de ROS2 sur la py
 -connection en huart entre la py et la stm32
 -possibilité d'un envoie de message depuis la py vers la stm32 pour controler des moteurs et définir les valeurs de vitesses souhaités
 -Possibilité de controler la base roulante avec le clavier
 -ROSBAG ( possibilité de rejouer une séquence faite avec le robot )
 -Lidar ( pour l'instant, il ya bien une évoltion dans le bon sens de la distance évaluée par le lidar lorsqu'un objet se déplace mais cette distance n'est pas encore très précise ) 
 
## Répartition par tâche :

## 1-ROS2/STM32 :
- Antoine
- Mateo
- Sammy

Le compte rendu de cette equipe se trouve dans le ReadMe.md du dossier Firmware :
https://github.com/AresEnsea/2425_Projet2A_AresCFR/blob/d943063ebb5853ef2ae5634fa93b110d4156a13e/2A/Firmware/README.md

## 2-Traitement Image :
- Khalid
- Mohamed
- Abderahmane

Le compte rendu de cette équipe est présent ici :
https://github.com/AresEnsea/2425_Projet2A_AresCFR/blob/d943063ebb5853ef2ae5634fa93b110d4156a13e/2A/Vision/README.md

## 3-PCB :
- Kenny
- Ewan

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

