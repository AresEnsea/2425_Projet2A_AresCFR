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
