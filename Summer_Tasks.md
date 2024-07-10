Liste des trucs à faire cet tété

PCB description :
L'objectif final du PCB est de faire une carte mere (arrondie ?) pour le 
robot du projet.

La carte mere permet l'alimentation et la laison des périphériques (capteurs, actionneurs...) 
au microcontrolleur (STM32H7xxxx) les commandants et permet également à la stm32 de communiquer
avec un raspberry pi qui ferait du traitement d'information ainsi que la communication vers l'extérieur 
via le WIFI.

L'alimentation des périphériques serait faite avec la tension d'alimentation du PCB mère pour minimiser
les pertes et serait donc adaptée sur les cartes de drivers que l'on doit également designer.

Pour l'instant l'idée est de faires des cartes auxiliaires différentes pour chaque actionneur.
Les cartes auxiliaires sont composée de drivers pour les différents capteurs et actionneurs si nécessaire
et d'un microcontrolleur STM32. Le tout serait lié à la carte mère via un faisceau contenant  :
  - une alimentation 14,4V (Tension la plus élevée pour minimiser les pertes dans la longueur de cable)
  - Deux fils torsadés pour assurler la liason I2C (voir la vitesse en fonction des capacités des microcontrolleur)
  - Un fil ground (pour la partie puissance vérifier les épaisseur de cable en fonction  de ce que tirent les actionneurs
en terme de courant.)

Avec ce système on peut avoir un nombre modulable d'appareils sur une même ligne I2C et ne ce soucier que de connecter 
les différents modules externes à la carte mère de la même manière que l'on ne se soucie que de brancher sa clef usb.

