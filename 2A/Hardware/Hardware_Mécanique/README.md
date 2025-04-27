# Rapport de l'équipe mécanique

![image](https://github.com/user-attachments/assets/89a816ae-5087-4a4a-9b37-609c87e18b05)

La mécanique à été réalisée par ROMEO Lorenzo, GARBEZ Matis, avec une petite participation de YOUSFI Bilal qui a voulu réaliser les fixations pour les moteurs stepper des poulies.
On peut voir sur la photo ci dessus la totalité du robot assemblé. On y trouve donc la base roulante sur lesquels sont montés les moteurs ainsi que l'odométrie et les PCBs. Les rails linéaires fixés aux tiges d'alluminium extrudé afin de supporter les mécanismes de pinces élévatrices s'élevant par un système de poulie actionné par moteurs stepper. Ces tiges supportent la plaque supérieure sur laquelle sont montés les moteurs stepper actionnant les poulies mais aussi le LiDAR pour l'évitement d'obstacles, le routeur et ses antennes pour communiquer avec la NVidia Jetson, le support de balise monté au dessus du LiDAR, le bouton d'arrêt d'urgence et le mécanisme d'activation par tirette actionné par dépression d'un switch en enlevant la tirette.

Par ailleurs vous pourrez remarquer que la plaque supérieure est bien plus petite que la plaque inférieure. Cela permet d'empiler 3 étages et donc de répondre entièrement aux contraintes imposées par la compétition, malgré le 3 ème étage étant plus haut que la hauteur maximale autorisée du robot.

Tous les fichiers 3D sont disponibles sur OnShape via ce lien : https://cad.onshape.com/documents/e7dd6126290aaac8861c9b96/w/d5c385b6723083ea50ea9e11/e/9022f36f38f961f67dfb7efc?renderMode=0&uiState=6800e786ed3eb5033177a7fb

## Base roulante

  Nous avons opté pour des moteur à courant continu MAXON, assemblé avec un réducteur et un encodeur afin de pouvoir développer un rapport couple-vitesse optimal et asservir la vitesse de rotation moteur.
  De plus nous avons redesigné le système d'accroche des encodeurs de roue ainsi que le système de billes porteuses. Ces èlèments sont désormais posés sur de ressorts ce qui nous a permis d'opter pour des roues de type compliant BaneBots procurant une excellente adhérence au robot facilitant le suivi de trajectoire.

![image](https://github.com/user-attachments/assets/215828df-55af-4cb2-b604-4adb616b72f2)
  
Voici donc ci dessus la base roulante finale de notre robot

![Capture d’écran du 2025-01-24 22-35-08](https://github.com/user-attachments/assets/35084065-b79e-4923-bfae-4ddd7a8b2774)

Vous pouvez observer ci dessus le système de suspensions de l'odométrie ainsi que des billes porteuses.

La projection verticale de cette plaque inférieure combinée à celle des mécanismes de pinces doit définir un périmètre de moins de 1200mm lorsque les mécanismes sont au plus compact et moins de 1400mm lorsque les mécanismes sont déployés. Nous avons bien fait attention à respecter cela, si bien que nous atteignont presque les 1200mm de périmètre lorsque le robot est entièrement déployé.

## Déploiement de la bannière

  Dans la compétition nous devons déployer une bannière de 50cm de long pour 25 cm de large. Nous pouvons la déployer sur le terrain ou contre la table. Pour éviter des problèmes avec le robot principal et les pamis, nous avons décidé de la déployer sur le rebord de la table. 
  Les règles imposent une taille maximal pour le robot, c'est pourquoi nous devions trouvé un mécanisme afin de pouvoir la plier en début de match et la déplier sur le rebord de la table. Dans un premier temps nous avions proposé un système totalement mécanique. Cependant, le déploiement n'était pas parfait et nous avions peur de ne pas répondre au critère de la compétition. C'est pourquoi nous avons optés pour un déploiement de la partie du haut de la banderolle mécanique et électronique avec un servo-moteur. 
  Ainsi la bannière sera déployée depuis un porte bannière, posé sur la pince du haut et poussé par les "bras" des pinces. On aura donc dans un premier temps la partie du bas de la bannière qui se déroulera par l'action de la pesanteur et du poids du mécanisme autour duquel la bannière est pliée puis enroulée. Ce dernier dépliera la bannière en fin de course grâce à la tension de ressorts comprimés par l'enroulement de la bannière.
  Nous avons opté pour ce système de déploiement car il reste relativement simple et efficace, il était nécessaire de plier ou d'enrouler la bannière car la réglementation impose une largeur et une longueur de bannière trop grandes pour etre stockées dans le robot comme tel.
  La partie du haut se déploiera par le même mécanisme mais uniquement quand le robot enverra la commande au servo moteur. Ainsi la bannière sera proprement déployeée. Le robot n'aura plus qu'à se déconnecté en roulant de l'autre côté de la bannière, ce qui déconnectera les pins header relié à la carte stm. 

![image](https://github.com/user-attachments/assets/be71b908-b71c-48ec-9c25-01bfbd73b470)

Vous pouvez voir ci dessus la partie qui sera accrochée sur le bord de l'aire de jeu. La bannière y sera attachée entre les deux différentes plaques maintenues par un système de vis écrou. Les rebords extrudés de la partie grise ainsi que du bas de l'accroche de la partie bleue serviront à mieux maintenir la bannière enroulée lors de l'accroche du porte bannière sur le rebord de la table, et autoriser ensuite la chute du mécanisme de déploiement. On peut également y voir le trou pour le servo moteur et celui où les fils sortiront. 



https://github.com/user-attachments/assets/2ac68d6e-2b8b-40fc-8861-738e08470789


![Capturevidodu24-01-2025230203-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/7a24c7ee-1fc4-43a6-92a6-2346f90275c3)

Ceci est une animation du système de déploiement de la bannière, il faut imaginer que tout comme pour le porte bannière, l'on attache la bannière en la serrant entre les plaques avec un système de vis écrous. Deux ressorts planaires de 180 degrés sont placées au sein des pivots afin d'exercer une force cherchant à déployer le mécanisme. Le mécanisme du bas est maintenu fermé en l'enroulant dans la bannière tendu que celui du haut est bloqué avec le servo-moteur.

## Pinces élévatrices

![image](https://github.com/user-attachments/assets/a73c80eb-1c25-46c3-9eb7-8602b1113a6b)

  Les mécanismes sont concus de sorte à transporter tout un étage de l'estrade comme tel et le déposer déjà monté. Il se base sur un mécansime à fils afin de resserer les pinces autour des deux "piliers" centraux de l'estrade et des bras viennent porter la planche supérieure de l'estrade. Ce mécanisme est monté sur un système de poulie et rail linéaire. Nous prévoyons deux mécanismes pour chacun des deux cotés du robot, soit la capacité à déplacer 4 estrades à la fois et aisni aisément construire deux estrades de deux etages.
De plus, à cela s'ajoute la possibilité de pouvoir porter deux etages déja assemblés avec une unique pince nous permettant donc de pouvoir empiler trois étages. Cela traduit donc une mécanique complète. Nous considérons qu'il ný a plus rien à améliorer d'un point de vue mécanique sur les pinces.
Le choix d'un mécanisme acctionné par des fils repose sur le fait de vouloir minimiser l'empreinte de la pince mais aussi le nombre d'actionneurs et donc de cablage et de STM32 à devoir gérer. De plus cela permet de développer tout de même une force importante et amplement suffisante pour porter meme plusieurs étages d'un seul coup.

![Capturevidodu24-01-2025234327-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/f864907f-14ca-48cb-84fd-f37cff8e279c)

Voici donc le fonctionnement des pinces principales actionnées par l'enrouleur central en gris et légérement déporté vers la droite actionné par un servomoteur KST, essentiel pour délivrer le couple nécessaire.

![Capturevidodu24-01-2025234354-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/c4e8f96a-34e9-41d9-8fe1-ef323870e8e1)

L'enrouleur central a été déporté afin d'autoriser le fonctionnement du mécanisme auxiliaire permettant de mieux englober la boite de conserve à soulever, assurant une prise à toute épreuve.

![Capturevidodu24-01-2025234951-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/324428de-3780-4965-b9b9-f4a586f2f41e)

Ce mécanisme est responsable de soulever les planches posées sur les boites de conserve. Un switch est intégré dans les bras porteurs afin de reconnaitre facilement lorsque le chariot est placé de sorte à pouvoir soulever la planche avec certitude; cela est une redondance à l'exécution de la maneuvre de récupération d'un étage étant donné que l'élévation par steppers permet déjà une connaissance assez précise de la position verticale du mécanisme:

1. Le robot s'aligne à l'étage présent sur la table.
2. Il fait monter le chariot élévateur jusqu'à ce que les bras atteignent les planches
3. Il reserre alors les pinces afin d'attraper les canettes et soulève la totalité de l'étage.

## Support PCB :

Support pour PCB moteur, servo-moteur et stepper :
![WhatsApp Image 2025-04-27 à 22 23 22_0c7d7915](https://github.com/user-attachments/assets/f32db625-77b3-48f3-b1bb-83cd14ad7141)

Dans le cas où nous utilisons les PCBs conçus, nous avons proposé une tour à PCB qui controlent tout les moteurs du robot. Cette tour a été pensé pour qu'elle puisse réunir un maximum de PCB de la manière la plus ergonomique possible. Les PCBs s'y fixent par encastration et le support est encastré dans le bois de la base roulante. Ils sont placés stratégiquements à cotés des moteurs de chaque côté du robot. Ces pcbs sont alimentés par le pcb multiprise :

![WhatsApp Image 2025-04-28 à 00 16 25_3da53bbe](https://github.com/user-attachments/assets/ebebe4a9-3a8b-46fd-878f-64203c316ff6)

Ce PCB est relié à un support qui reprend sa forme et qui installé dans le robot sur un rail afin de pouvoir ajuster sa position en cas de besoin. 

## Support antenne :

Le plus simple avec la place qu'il nous reste est de placé le routeur et les antennes au dessus du robot :

![image](https://github.com/user-attachments/assets/102ca89f-f7d4-46bf-9fa6-8a2e19974755)

Elles sont proche du routeur fixé par ses mounting holes sur la plaque du haut. Un angle de 90° est mis entre les antennes comme demandé par l'équipe réseau. 

Dans l'ensemble nous avons répondu aux demandes de chacune des équipes tout en respectant les règles de la compétition. Le robot est assemblé !




