# Rapport de fin de semestre :

Durant ce semestre notre équipe à pu réaliser différents PCB : 

## PCB_ MotherBoard

![image](https://github.com/user-attachments/assets/7895d0b8-4d78-4fce-9e91-588a723b96fe)

### Description : 
 - La carte mère permet l'alimentation et la laison des péripériques (capteurs, actionneurs...) 
au microcontrolleur (STM32H735) les commandants et permet également à la stm32 de communiquer
avec un raspberry pi qui ferait du traitement d'information ainsi que la communication vers l'extérieur 
via le WIFI.

### Features Principales: 
 - 9 liasons inter cartes : connecteur JST4 (1 UART + alimentation 24V).
 - Shield pour Raspberry Pi assurant l'alimentation 5V ainsi qu'une liason UART.



## PCB_Moteur V2
![image](https://github.com/user-attachments/assets/984a2635-1796-419c-b235-47eec7ee183f)
### Descriptopn :
 - Carte de pilotage des moteurs de la base roulante. Nous avons prévu une carte par moteur.


### Caractéristiques techniques : 
- Supporter les appels de courants jusqu'à 10A par moteur.
- Connction des sondes à effet hall du moteur en RS323.
- Connection du moteur en XT30.
- STM32G474 embarquée pour communiquer avec la carte mère ainsi que les différents péripériques de la carte.
- Capteur de Température.
- Sondes à effet Hall pour la mesure de courant instantané consommé par le moteur.
- Possibilté de brancher un ventilateur.
  
### Issues :

-  La V1 du PCB n'a pas été concluante suite au non fonctionnement d'un convertisseur buck. Celui-ci à été changé dans la V2.


# PCB_Stepper 
![image](https://github.com/user-attachments/assets/bfb7ffd5-f9a2-40f6-af61-cffc47343d7f)
## Description :
 - Carte de pilotage pour 2 steppers steppers.

## Caractéristiques principales : 
 - Supporte l'eau la plie les pets et le vent grâce aux 2 TMC2225 embarqué pour le pilotage des steppers.
 - STM32G071 embarquée pour la liason UART avec la carte mère ainsi que la commande des moteurs...

## Issues :

- V1 actuelle .
- Pour la V2 ce serait bien de mesure de courant en instantané.












