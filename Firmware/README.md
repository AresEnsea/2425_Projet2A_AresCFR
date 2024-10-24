Ici le dossier Firmware du Github de la Coupe de France 2025 d'Ares, il regroupe les fichiers C,C++ et Python qui seront utilisés pour programmer le robot principal.

- CFR_G474RET6 : le projet STM32CubeIDE que nous implémenteront ultérieurement dans le microprocesseur (G474RET6) du pcb, pour le moment il contient simplement le pinout
- STM32_Pi_Connexion : contient le projet qui STM32CubeIDE qui permet la connexion UART entre une STM32CubeIDE L476RG et une raspberry pi4
- STM32_moteurs : contient le projet STM32CubeIDE qui délivre deux PWM pour faire tourner des moteurs
- ros2_ws : le workspace ROS actuellement sur la pi4 (booté avec Ubuntu 22.04 et utilisant ROS2)


