# PCB_Servo
![image](https://github.com/user-attachments/assets/0d86c332-15f9-4467-bbf8-0971174e202c)


## Description :
- Carte servant d'intermédiaire entre la mainboard et tous les servos du robot

## Caractéristiques techniques :
- Le connecteur JST 4 pins recoit la puissance et une information UART de la carte mainboard
- La puissance est convertie en 7.4V pour alimenter les servos
- Le 7.4V est convertie en 3.3V pour alimenter le microprocesseur STM32G071CBUx
- L'information UART est traduite par le microprocesseur en un signal PWM par servo

# Issues
- le modèle de servo moteur devant etre utilisé à changé après avoir finis le routage, il a fallu alors reprendre le routage de toute la puissance de la carte
