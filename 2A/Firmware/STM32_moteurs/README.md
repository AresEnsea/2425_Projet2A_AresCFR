 -Ces directory s'utilisent avec les projets ROS associés qui gère l'UART, le premier permet d'envoyer simplement un 0 ou un 1 le deuxième permet de choisir le nombre de caractères dans le message UART (par défaut 4), il suffit dans le code STM32 de modifier la taille de "rx_data", par défaut à 7 pour 4 caractères dans le messsage, augmenter rx_data avec le nombre de caractère envoyés (4-7,5-8,6-9, etc).

-cfr_V1 : version finale du contrôle des moteurs en fonction de ce qu'il reçoit de la raspberry pi.

## Troubleshouting : 
Si problème en reprenant un projet stm32, penser à regénérer un binary (.elf) en suivant les instructions suivantes : 
![image](https://github.com/user-attachments/assets/b214b902-5b55-42d8-9a51-c7b45a9483c1)
