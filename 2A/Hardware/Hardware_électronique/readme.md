# Rapport de fin de semestre :

Durant ce semestre notre équipe à pu réaliser différents PCB...

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

### Issues :
 - Malheuresement, suite à une erreur d'empreinte, le PCBMotherBoard 



## PCB_Moteur V3
![image](https://github.com/user-attachments/assets/b463e2cb-5f65-497a-8cfb-58d749f0a646)

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
-  La V2 à fonctionné mais avec un problème qui faisairt que le moteur ne touranit que dans un sens. La V3 rajoute une resistance de sense entre le bras de pont et le mosfet driver. 


## PCB_Stepper 
![image](https://github.com/user-attachments/assets/bfb7ffd5-f9a2-40f6-af61-cffc47343d7f)
### Description :
 - Carte de pilotage pour 2 steppers steppers.

### Caractéristiques principales : 
 - Supporte l'eau la plie les pets et le vent grâce aux 2 TMC2225 embarqué pour le pilotage des steppers.
 - STM32G071 embarquée pour la liason UART avec la carte mère ainsi que la commande des moteurs...

### Issues :

- V1 actuelle .
- Pour la V2 ce serait bien de mesure de courant en instantané.


# PCB_Servo
![IMG_7566](https://github.com/user-attachments/assets/c269c726-2128-440c-a93d-ea61964eadca)


## Description :
- Carte servant d'intermédiaire entre la mainboard et tous les servos du robot

## Caractéristiques techniques :
- Le connecteur JST 4 pins recoit la puissance et une information UART de la carte mainboard.
- La puissance est convertie de 25.2V en 7.4V pour alimenter les servos.
- Le 7.4V est convertie en 3.3V pour alimenter le microprocesseur STM32G071CBUx.
- L'information UART est traduite par le microprocesseur en un signal PWM par servo.

## Finalité:
Quatre PCB servos furent soudés. 
Trois sont opérationnels, cela nous permet d'en avoir deux dans le robot et un en réserve par sécurité.
En effet, un PCB servo peut commander 8 servos et il y a 12 servos à commander à la fois sur le robot.
Le troisième constitue une sécurité au cas ou une des deux cartes utilisées viendrait à casser (court-circuit, surcharge, choque...).

# Issues
- Le modèle de servo moteur devant etre utilisé à changé après avoir finis le routage, il a fallu alors reprendre le routage de toute la puissance de la carte
- Sur les quatre PCB soudés, tous ne fonctionnent pas parfaitement:
   Je ne suis pas parvenu à téléverser un quelconque programme sur le microprocesseur de la première carte que j'ai soudé. Celle-ci est donc considéré comme HS.
   La deuxième carte est opérationnelle mais je lui ai relevé deux défaults. Premièrement, elle délivre du 8,2V aux servos au lieu de 7,6V. Deuxièmement, je n'ai réussis qu'à souder 6 connecteurs JST pour relier la carte aux servos.
   Les deux cartes soudées en dernières fonctionnent parfaitement (aucun problème de fonctionnement relevé).
  
