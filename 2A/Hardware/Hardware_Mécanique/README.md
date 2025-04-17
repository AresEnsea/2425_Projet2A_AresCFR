# Rapport de l'équipe mécanique

![image](https://github.com/user-attachments/assets/89a816ae-5087-4a4a-9b37-609c87e18b05)

La mécanique à été réalisée par ROMEO Lorenzo, GARBEZ Matis, avec une petite particpatioon de YOUSFI Bilal qui a voulu réaliser les fixations pour les moteurs stepper des poulies.

## Base roulante

  Nous avons opté pour des moteur à courant continu MAXON, assemblé avec un réducteur et un encodeur afin de pouvoir développer un rapport couple-vitesse optimal et asservir la vitesse de rotation moteur.
  De plus nous avons redesigné le système d'accroche des encodeurs de roue ainsi que le système de billes porteuses. Ces èlèments sont désormais posés sur de ressorts ce qui nous a permis d'opter pour des roues de type compliant BaneBots procurant une excellente adhérence au robot facilitant le suivi de trajectoire.

![image](https://github.com/user-attachments/assets/215828df-55af-4cb2-b604-4adb616b72f2)
  
Voici donc ci dessus la base roulante finale de notre robot

![Capture d’écran du 2025-01-24 22-35-08](https://github.com/user-attachments/assets/35084065-b79e-4923-bfae-4ddd7a8b2774)

Vous pouvez observer ci dessus le système de suspensions de l'odométrie ainsi que des billes porteuses.

## Déploiement de la bannière

  La banière sera déployée depuis un porte bannière qui sera accroché par le robot sur le bord de l'aire de jeu, la bannière sera ensuite déroulée par l'action de la pesanteur et du poids du mécanisme autour duquel la bannière est pliée puis enroulée, ce dernier dépliera la bannière en fin de course par la tension de ressorts comprimés par l'enroulement de la bannière.
  Nous avons opté pour ce système de déploiement car il reste relativement simple et efficace, il était nécessaire de plier ou d'enrouler la bannière car la réglementation impose une largeur et une longueur de bannière trop grandes pour etre stockées dans le robot comme tel.

![image](https://github.com/user-attachments/assets/7d14aec5-ddd7-40d0-9cb9-9021474d7b22)

Vous pouvez voir ci dessus la partie qui sera accrochée sur le bord de l'aire de jeu. La bannière y sera attachée entre les deux différentes plaques maintenues par un système de vis écrou. Les rebords extrudés de la partie grise ainsi que du bas de l'accroche de la partie bleue serviront à mieux maintenir la bannière enroulée lors de l'accroche du porte bannière sur le rebord de la table, et autoriser ensuite la chute du mécanisme de déploiement.

![Capturevidodu24-01-2025230203-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/7a24c7ee-1fc4-43a6-92a6-2346f90275c3)

Ceci est une animation du système de déploiement de la bannière, il faut imaginer que tout comme pour le porte bannière, l'on attache la bannière en la serrant entre les plaques avec un système de vis écrous. Deux ressorts planaires de 180 degrés sont placées au sein des pivots afin d'exercer une force cherchant à déployer le mécanisme. Le mécanisme est maintenu femré en l'enroulant dans la bannière.

## Pinces élévatrices

![image](https://github.com/user-attachments/assets/1db8e8d8-f833-4263-b094-adb31e3e23de)

  Les mécanismes sont concus de sorte à transporter tout un étage de l'estrade comme tel et le déposer déjà monté. Il se base sur un mécansime à fils afin de resserer les pinces autour des deux "piliers" centraux de l'estrade et des bras viennent porter la planche supérieure de l'estrade. Ce mécanisme est monté sur un système de poulie et rail linéaire. Nous prévoyons deux mécanismes pour chacun des deux cotés du robot, soit la capacité à déplacer 4 estrades à la fois et aisni aisément construire deux estrades de deux etages.
Le choix d'un mécanisme acctionné par des fils repose sur le fait de vouloir minimiser l'empreinte de la pince mais aussi le nombre d'actionneurs et donc de cablage et de STM32 à devoir gérer.

![Capturevidodu24-01-2025234327-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/f864907f-14ca-48cb-84fd-f37cff8e279c)

Voici donc le fonctionnement des pinces principales actionnées par l'enrouleur central en gris et légérement déporté vers la droite actionné par un servomoteur KST.

![Capturevidodu24-01-2025234354-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/c4e8f96a-34e9-41d9-8fe1-ef323870e8e1)

L'enrouleur central a été déporté afin d'autoriser le ce mécanisme auxiliaire permettant de mieux englober la boite de conserve à soulever.

![Capturevidodu24-01-2025234951-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/324428de-3780-4965-b9b9-f4a586f2f41e)

Ce mécanisme est responsable de soulever les planches posées sur les boites de conserve. Un switch est intégré dans les bras porteurs afin de reconnaitre facilement lorsque le chariot est placé de sorte à pouvoir soulever la planche avec certitude; cela est crucial à l'exécution de la maneuvre de récupération d'un étage:

1. Le robot s'aligne à l'étage preésent sur la table.
2. Il fait monter le chariot élévateur jusqu'à ce que les interrupteurs de fin de course s'actionnent
3. Il reserre alors les pinces afin d'attraper les canettes et soulève la totalité de l'étage.

