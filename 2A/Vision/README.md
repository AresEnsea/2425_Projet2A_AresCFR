# Vision - Projet 2A Ares CFR

Ce dépôt contient les travaux de l'équipe Vision pour le projet 2A Ares CFR, dans le cadre de la Coupe de France de Robotique. 

L'objectif est de développer un système de vision performant permettant :
1. La reconnaissance d'éléments et marqueurs dans un environnement robotique.
2. Le calcul de chemins optimaux pour le robot en intégrant les contraintes dynamiques du terrain de jeu.

---

## 1. Calibration de la Caméra

### Contexte et Méthodologie
La calibration de la caméra est une étape cruciale pour corriger les distorsions optiques et garantir la précision des mesures. 
Elle a été réalisée en utilisant un échiquier standard avec ROS et OpenCV. Les principales étapes suivies étaient :
- Capture de plusieurs images de l'échiquier à partir de différents angles.
- Détection automatique des coins de l'échiquier dans chaque image.
- Calcul des paramètres intrinsèques et extrinsèques à l'aide de la méthode de régression linéaire.
- Validation des résultats à l'aide d'un modèle de reprojection.

### Résultat
- Calibration validée sur 50 images d’un échiquier avec une faible erreur RMS.
- Les paramètres calculés incluent :
  - Une matrice intrinsèque précise pour la caméra.
  - Des coefficients de distorsion réduisant les aberrations.
- Tests validés sur la maquette et dans des conditions réelles.

---

## 2. Détection des ArUco

### Contexte et Règlement
Le règlement de la compétition impose l’utilisation de marqueurs ArUco fixes sur le terrain de jeu. Ces marqueurs servent de points 
de référence pour le positionnement du robot. Nous avons conçu un pipeline complet pour exploiter ces ArUcos et fournir des données 
précises sur la localisation et l’orientation du robot.

### Méthodologie
- Détection des marqueurs grâce à la bibliothèque OpenCV.
- Extraction des IDs des ArUco et calcul des vecteurs de translation et de rotation.
- Validation des résultats par des tests comparatifs avec des données réelles.
- Robustesse testée dans différents environnements avec des niveaux d’éclairage variés.

### Implémentation et Résultats

#### Détection des ArUco sur une maquette
- Le système détecte les ArUco avec leurs orientations.
- Extraction précise des vecteurs de translation et de rotation pour chaque marqueur.

#### Détection en conditions réelles
- Robustesse démontrée sur le terrain de jeu, malgré des perturbations visuelles (objets et ombres).
- Optimisation des paramètres de détection pour minimiser les erreurs.

---

## 3. Navigation et Cheminement

### Positionnement et Calcul de Chemins

#### Position Initiale et Représentation 2D
- Les ArUco fixes servent de repères pour définir l'origine et la position initiale du robot.
- Un modèle mathématique est utilisé pour convertir les coordonnées du repère ArUco en une représentation 2D utilisée par le système 
de navigation.
- Intégration avec un algorithme de discrétisation pour diviser le terrain en zones accessibles et inaccessibles.

#### Suivi du Mouvement et Ajustements
- Les vecteurs de rotation et de translation des ArUco sont utilisés pour mettre à jour les positions.
- Détection d’une erreur dans le calcul des distances : la distance parcourue est sous-estimée, ce qui nécessite un ajustement 
des coefficients.

---

## 4. Infrastructure du Système

### Tour pour la Caméra
En attendant la tour définitive conçue par l'équipe mécanique, une solution temporaire a été mise en place pour fixer la caméra. 
Cette tour temporaire permet une flexibilité suffisante pour les tests tout en offrant une stabilité correcte pour la capture 
des données visuelles.

---

## 5. Calcul de l'Erreur de Position et Performances

### Méthode
Pour évaluer la précision du système de vision, une comparaison a été effectuée entre la position réelle du robot et celle calculée par notre code. Cette évaluation a permis de déterminer l'erreur de localisation dans le contexte de la détection des ArUco.

### Résultats
- Erreur de Position : L'erreur moyenne observée dans la position calculée par le système est d'environ 1.8 cm maximum.
- FPS (Frames Per Second) : 
  - Setup Normal : Entre 3 et 4 FPS, ce qui est suffisant pour une détection basique mais pourrait être amélioré pour des scénarios dynamiques.
  - Setup Optimisé : Entre 10 et 12 FPS, ce qui offre une fluidité de détection plus élevée pour une meilleure réactivité du robot.

### Validation
Les résultats obtenus respectent les exigences de précision et de performance définies dans le cahier des charges de l'équipe ROS, avec une erreur de localisation bien en dessous de la limite autorisée et un taux de FPS adapté selon les configurations.

---

## 6. Vidéo du Système en Action

Une vidéo a été réalisée pour illustrer les performances du système de vision et la détection en temps réel des ArUco. Cette vidéo montre la réactivité du robot et la précision de la localisation dans différents scénarios de terrain.

Lien de la vidéo :  
https://github.com/user-attachments/assets/b6591760-27ca-40ef-97e6-4b8968a7495c

---

## 7. Conclusion

Le système de vision développé a atteint les objectifs de précision et de performance définis dans le cadre du projet 2A Ares CFR. Grâce à l'intégration des ArUco pour la détection de position et de rotation, ainsi qu'à l'optimisation des FPS, nous avons obtenu des résultats fiables et robustes.

---

### Équipe Vision
- Khalid ZOUHAIR  
- Mohamed EL KOURMISS  
- Abderrahmane EL FELSOUFI
