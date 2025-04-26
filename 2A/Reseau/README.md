# Rapport sur la partie réseau

Ce dossier réseau permet de conserver une copie des backups les plus récents pour les 2 routeurs utilisés pendant la coupe de France de robotique ( édition de 2025 )
# Objectif

Permettre à la partie vision de transmettre les informations recueillies des différents éléments de la table et de les transmettre au robot mobile ( voire aux pamis ) mais de façon isolée par un wifi local. Pour ce faire nous allons utiliser deux routeurs, l'un sur la base fixe avec la caméra et l'autre sur le robot principal.

![image](https://github.com/user-attachments/assets/8db067f3-1d20-40cb-924a-8088c3e1e051)


# Matériel

## Routeurs de chez MikroTik

![image](https://github.com/user-attachments/assets/52a52599-99d8-4636-91de-8589f043f12c)


## WinBox 3.41​
Le logiciel utilisé pour administrer les différents routeurs, voici son interface :
![image](https://github.com/user-attachments/assets/52d40e52-e719-4838-92b8-09766d16cc4e)


# Travail effectué

## Initialisation des routeurs

Mise en place d'un bridge entre l'interface wlan et ethernet de chaque routeur pour autoriser la communication entre ces deux interfaces. Mise en place des IP statiques de chaque routeur 192.168.88.10 pour le routeur de base ( routeur 1 ) et 192.168.88.20 pour le routeur "robot" ( routeur 2 ).

## Mise en place d'un wifi généré de façon locale

Le routeur 1 a été configuré pour émettre un wifi "wifi_from_base" et le routeur 2 a été paramétré pour s'y connecter. (si problème cf Table des problèmes plus bas)

## Configuration supplémentaire

Le wifi du routeur 1 utilise un serveur DHCP qui attribue des adresses à quiconque s'y connecte initialement prévu pour être dans la plage 192.168.88.100 - 192.168.88.199 j'ai restreint ce pool à 192.168.88.100 - 192.168.88.149 car il y avait des problèmes d'adressage d'ip, et que 50 ip devraient être largement suffisantes pour notre réseau local. Les ip devraient également s'attribuer dans le sens croissant.

Les tables de routages ont été configurées pour permettre la connection entre les connexions du routeur 2 aux connections du routeur 1

## Test de ping entre les routeurs
### Schéma de test : 
![image](https://github.com/user-attachments/assets/8c607225-5242-451a-ac1b-c985963212d1)

### On essaye avec le pc1 de ping le routeur 1 puis le routeur 2 : 
![1737761697386](https://github.com/user-attachments/assets/72c4ea1d-6afc-4004-ba3c-4e680dbdf299)

### On essaye avec le pc2 de ping le routeur 2 puis le routeur 1 : 
![1737761697379](https://github.com/user-attachments/assets/480c22ab-a2ad-41c6-b392-3b57c32a75be)

Les deux tests ont été réussis, les pings ont bien été envoyés et reçus ce qui prouve bien la connexion.

## Mise en place d'un DNS local

test d'attribution pour un ordinateur connecté en ethernet derrière le routeur 1
![image](https://github.com/user-attachments/assets/1dface88-ba38-4399-b58a-a6c75af169de)

Depuis l'ordinateur connecté au réseau : 
![image](https://github.com/user-attachments/assets/fbfdb7e1-c06d-4251-8d11-0e7d8e46daa6)

 Attention : Avec l'ordinateur connecté en wifi il est possible de ping un pc connecté en ethernet au routeur, mais pour faire la manipulation inverse il était nécessaire de désactiver le pare feu windows (information à retenir pour la suite)

### Remarque :

Nous abandonnons la partie avec le switch physique nous permettant de se connecter à internet depuis le réseau local n'étant pas sûr d'avoir un câble RJ45 relié au réseau sur les lieux de la CFR, éventuellement nous pouvons faire des profils supplémentaires si le cas se présente

## Table des problèmes : 
### Impossible de se connecter au routeur ?
 -> assurez vous d'être sur le même réseau que le routeur : 
 
 -soit connectez vous à son wifi (s'il en émet un)
 
 -soit vous vous connectez à une box internet avec le routeur connecté en RJ45 dessus. Référez vous aux bonnes adresses mac présentes sur les routeurs si vous passez par une connexion filaire ou sans fil
### Problème de connexion entre le pc de commande et le reste :
 -> désactiver pare-feu 
### Problème de connexion entre les deux routeurs / Impossible de se connecter au wifi du routeur base avec PC :
 -> vérifier les bandes de fréquence du routeur base et du routeur robot pour qu'elles soient identiques
 
 -> pour la connexion avec le pc (du moins avec le mien) il faut utiliser le paramètre 20/40/80 Hz dans les bandes de fréquence du wifi avec comme paramètre de fréquence de 5GHz AC et non AX (qui correspondent respectivement au wifi 5 et au wifi 6)
### Moindre problème ?
 -> Ne pas hésiter à revenir sur une version précédente grâce aux backups localement enregistrés sur les routeurs (une version est aussi disponible pour le routeur base sur ce répertoir git), il faut aller dans la rubrique "file" à gauche dans winbox pour y accéder.
### Pour réinitialiser manuellement le routeur : 
 1. Couper son alimentation
 2. Maintenir le bouton reset
 3. Alimenter à nouveau le routeur en maintenant ce bouton enfoncé
 4. une fois que les LED commencent à clignoter, vous pouvez relâcher le bouton et attendre la réinitialisation du routeur
## Equipe réseau : 

### Respo réseau : Nathan
