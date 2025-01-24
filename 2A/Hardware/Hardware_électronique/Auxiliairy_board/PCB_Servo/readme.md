
![image](https://github.com/user-attachments/assets/a935942f-0da9-4f4b-93e9-5e0f482d999f)

# PCB_Servo

## Description :
- Carte servant d'intermédiaire entre la mainboard et tous les servos du robot

## Caractéristiques techniques :
- Le connecteur JST 4 pins recoit la puissance et une information UART de la carte mainboard
- La puissance est convertie en 7.4V pour alimenter les servos
- Le 7.4V est convertie en 3.3V pour alimenter le microprocesseur STM32G071CBUx
- L'information UART est traduite par le microprocesseur en un signal PWM par servo

# Issues
- le modèle de servo moteur devant etre utilisé à changé après avoir finis le routage, il a fallu alors reprendre le routage de toute la puissance de la carte
