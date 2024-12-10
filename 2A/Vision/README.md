# Vision - Projet 2A Ares CFR

Ce dépôt contient les travaux de l'équipe Vision dans le cadre du projet 2A Ares CFR. L'objectif est de développer un système de vision embarqué performant pour la reconnaissance d'éléments dans un environnement robotique et de trouver des chemins optimaux en utilisant ROS.

---

## 📖 **Table des matières**
1. [Calibration de la Caméra](#calibration-de-la-caméra)
2. [Configuration et Système](#configuration-et-système)
3. [Navigation et Recommandations](#navigation-et-recommandations)
4. [Travail Effectué](#travail-effectué)
5. [Prochaines Étapes](#prochaines-étapes)
6. [Équipe](#équipe)

---

## 📸 **Calibration de la Caméra**

### **Contexte**
- La calibration de la caméra est une étape clé pour corriger les distorsions et garantir la précision des mesures.

### **Résultats**
- **Calibration complétée** avec succès grâce à 50 images d’un échiquier.
- Paramètres générés :
  - Matrice intrinsèque.
  - Coefficients de distorsion.
- Tests validés sur la maquette et en conditions réelles.

---

## ⚙️ **Configuration et Système**

### **Jetson Nano**
- Installation et configuration complètes de la Jetson Nano.
- Mise en place de **ROS 2**, qui sera utilisé pour la gestion des données et la navigation autonome.

---

## 🛤️ **Navigation et Recommandations**

### **Implémentation Actuelle**
1. Discrétisation de la carte :
   - Création d’une matrice contenant des **0** (zones accessibles) et des **1** (zones inaccessibles).
2. Matrice des coûts :
   - Génération d'une matrice des coûts pour évaluer les distances en fonction des obstacles.
3. Algorithme de chemin optimal :
   - Utilisation de l’algorithme **A*** basé sur la distance de Manhattan.
   - Hypothèse : le robot ne peut se déplacer que dans quatre directions.

### **Recommandations des Professeurs**
Lors de la 3ᵉ présentation, il a été conseillé de :
- Passer à l’utilisation directe de **ROS** pour déterminer les chemins optimaux.
- Remplacer l'approche actuelle par une solution ROS nécessitant une **carte sous forme d'image en niveaux de gris**.

### **Progrès Actuel**
L’équipe travaille sur :
- Une méthode pour générer une carte réaliste du terrain de jeu intégrant :
  - Notre robot.
  - Le robot adverse.
  - Les zones de construction et autres éléments.
- Défi majeur : obtenir cette carte en temps réel à partir des données des caméras.

---

## ✅ **Travail Effectué**

1. Finalisation de la calibration de la caméra.
2. Configuration complète de la Jetson Nano avec ROS 2.
3. Développement d'un système de navigation basé sur :
   - Discrétisation de la carte.
   - Matrice des coûts.
   - Algorithme A* utilisant la distance de Manhattan.

---

## 🚀 **Prochaines Étapes**

1. Développer un pipeline pour convertir les données des caméras en une **carte en niveaux de gris**.
2. Intégrer la carte générée dans ROS pour déterminer des chemins optimaux avec une plus grande précision.
3. Optimiser le système pour un fonctionnement en temps réel dans des environnements dynamiques.

---

## 👥 **Équipe**
- **Khalid ZOUHAIR**
- **Mohamed EL KOURMISS**
- **Abderhamane EL FELSOUFI**

