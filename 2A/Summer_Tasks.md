# Liste des devoirs pour cet été :

## Description du PCB :
L'objectif final du PCB est de réaliser une carte mère (arrondie ?) pour le robot du projet.

La carte mère permet l'alimentation et la liaison des périphériques (capteurs, actionneurs...) au microcontrolleur (STM32H7xxxx) les commandant et permettant également à la stm32 de communiquer avec un Raspberry Pi (ou NVidia Jetson) qui ferait du traitement d'information (Images ou LiDAR) ainsi que la communication vers l'extérieur via le WIFI (ordi relié à 4 caméras aux coins de la table).

L'alimentation des périphériques se fera avec la tension d'alimentation du PCB mère pour minimiser les pertes et serait donc adaptée sur les cartes de drivers que l'on doit également projeter avec un convertisseur buck boost.

Pour l'instant l'idée est de faire des cartes auxiliaires différentes pour chaque actionneur.
Les cartes auxiliaires sont composée de drivers pour les différents capteurs et actionneurs si nécessaire et d'un microcontrolleur STM32. Le tout sera lié à la carte mère via un faisceau contenant :
  - une alimentation 14,4V (tension la plus élevée pour minimiser les pertes dans la longueur de cable)
  - Deux fils torsadés pour assurer la liason UART 
  - Un fil GND (pour la partie puissance vérifier l'épaisseur de cable en fonction du courant que tirent les actionneurs)

Avec ce système nous pouvons avoir un nombre modulable d'appareils sur une différents cannaux UART et ne se soucier que de connecter les différents modules externes à la carte mère de la même manière que l'on ne se soucie que de brancher sa clé USB.

