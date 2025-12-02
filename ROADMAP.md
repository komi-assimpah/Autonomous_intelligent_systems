# 🚀 Roadmap: Robot de Recherche d'Objets avec TurtleBot3

## 📋 Vue d'ensemble du projet

Vous allez créer un système autonome où le TurtleBot3 :
1. **Navigue** dans un environnement simulé (Gazebo)
2. **Diffuse** le flux vidéo de sa caméra sur un topic ROS2
3. **Détecte** un objet spécifique (ex: un chien) via vision par ordinateur
4. **S'arrête** automatiquement quand l'objet est trouvé

---

## 🎯 Phase 1: Comprendre les Bases de ROS2 (1-2 heures)

### Objectif
Comprendre comment ROS2 fonctionne avant de coder.

### Étapes pratiques

#### 1.1 Tester votre installation TurtleBot3
```bash
# Vérifier que ROS2 est bien configuré
ros2 topic list

# Lancer la simulation Gazebo avec TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Ce que vous devez observer** : Une fenêtre Gazebo s'ouvre avec un robot TurtleBot3 dans un environnement 3D.

#### 1.2 Explorer les topics disponibles
```bash
# Dans un nouveau terminal, lister tous les topics actifs
ros2 topic list

# Voir les messages de la caméra (si disponible)
ros2 topic echo /camera/image_raw --no-arr

# Voir la position du robot
ros2 topic echo /odom
```

**Concepts clés à comprendre** :
- **Node** : Un programme qui fait une tâche (ex: contrôler le robot, traiter l'image)
- **Topic** : Un canal de communication (ex: `/camera/image_raw` pour les images)
- **Message** : Les données envoyées sur un topic (ex: une image, une position)

#### 1.3 Contrôler le robot manuellement
```bash
# Lancer le contrôle par clavier
ros2 run turtlebot3_teleop teleop_keyboard
```

**Exercice** : Déplacez le robot avec les touches du clavier pour comprendre comment il bouge.

---

## 🛠️ Phase 2: Premier Package - Navigation avec Caméra (3-4 heures)

### Objectif
Créer un package qui fait naviguer le robot et publie le flux caméra.

### 2.1 Créer la structure du package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python object_search_navigation \
  --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
```

**Explication** :
- `object_search_navigation` : nom de votre package
- `rclpy` : bibliothèque Python pour ROS2
- `sensor_msgs` : pour les messages de caméra
- `geometry_msgs` : pour contrôler le mouvement
- `cv_bridge` : pour convertir les images ROS en OpenCV

### 2.2 Structure du code à créer

Vous allez créer **un seul node** qui fait deux choses :
1. Fait bouger le robot (navigation simple)
2. Publie les images de la caméra

**Fichier à créer** : `object_search_navigation/navigation_node.py`

**Fonctionnalités** :
- S'abonne à `/camera/image_raw` (reçoit les images)
- Republie sur `/search/camera_feed` (pour le détecteur)
- Publie sur `/cmd_vel` (pour faire bouger le robot)
- Implémente une stratégie de recherche simple (ex: tourner en spirale)

### 2.3 Compiler et tester

```bash
cd ~/ros2_ws
colcon build --packages-select object_search_navigation
source install/setup.bash
ros2 run object_search_navigation navigation_node
```

---

## 🔍 Phase 3: Deuxième Package - Détection d'Objets (4-5 heures)

### Objectif
Créer un package qui analyse les images et détecte l'objet cible.

### 3.1 Créer le package détecteur

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python object_detector \
  --dependencies rclpy sensor_msgs std_msgs cv_bridge
```

### 3.2 Choisir une méthode de détection

**Pour débuter (recommandé)** :
- Détection par couleur avec OpenCV (simple, rapide à implémenter)
- Exemple : détecter un objet rouge/bleu/vert

**Pour aller plus loin** :
- YOLO (détection d'objets réels comme "chien", "bouteille")
- Nécessite un modèle pré-entraîné

### 3.3 Structure du code

**Fichier à créer** : `object_detector/detector_node.py`

**Fonctionnalités** :
- S'abonne à `/search/camera_feed`
- Analyse chaque image reçue
- Publie sur `/search/status` : "SEARCHING" ou "STOP"
- Affiche l'image avec détection (optionnel, pour debug)

### 3.4 Tester le détecteur seul

```bash
# Terminal 1 : Lancer Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 : Lancer le détecteur
ros2 run object_detector detector_node --ros-args -p target_object:="chien"

# Terminal 3 : Vérifier les messages
ros2 topic echo /search/status
```

---

## 🔗 Phase 4: Intégration Complète (2-3 heures)

### Objectif
Faire fonctionner les deux packages ensemble.

### 4.1 Modifier le node de navigation

Ajouter un **subscriber** au topic `/search/status` :
- Si le message est "STOP" → arrêter le robot
- Sinon → continuer la recherche

### 4.2 Créer un launch file

**Fichier** : `object_search_navigation/launch/search_system.launch.py`

Ce fichier lance automatiquement :
1. La simulation Gazebo
2. Le node de navigation
3. Le node de détection

```bash
ros2 launch object_search_navigation search_system.launch.py target:="chien"
```

### 4.3 Tests complets

**Scénario de test** :
1. Le robot démarre et commence à chercher
2. Placer un objet cible dans Gazebo (ou utiliser un objet existant)
3. Le robot détecte l'objet et s'arrête
4. Vérifier les logs et les topics

---

## 🎨 Phase 5: Améliorations (Optionnel)

### 5.1 Évitement d'obstacles
- S'abonner au topic `/scan` (Lidar)
- Détecter les obstacles et les contourner

### 5.2 Stratégie de recherche intelligente
- Cartographie de l'environnement (SLAM)
- Planification de chemin optimale

### 5.3 Interface utilisateur
- Créer un node avec interface graphique (rqt)
- Afficher la caméra et le statut en temps réel

---

## 📚 Ressources d'apprentissage

### Documentation officielle
- [ROS2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

### Concepts clés à maîtriser
1. **Publishers & Subscribers** : Communication entre nodes
2. **Messages** : Types de données (Image, Twist, String, etc.)
3. **Launch files** : Démarrer plusieurs nodes ensemble
4. **Parameters** : Configurer vos nodes (ex: nom de l'objet cible)

---

## ✅ Checklist de progression

- [ ] Phase 1 : Simulation TurtleBot3 fonctionne
- [ ] Phase 1 : Je comprends les topics et messages
- [ ] Phase 2 : Package navigation créé
- [ ] Phase 2 : Robot se déplace et publie caméra
- [ ] Phase 3 : Package détection créé
- [ ] Phase 3 : Détection d'objet fonctionne
- [ ] Phase 4 : Système complet intégré
- [ ] Phase 4 : Robot s'arrête quand objet trouvé
- [ ] Phase 5 : Améliorations ajoutées (optionnel)

---

## 🚦 Prochaine étape

**Commencez par la Phase 1** : Testez votre installation et familiarisez-vous avec ROS2.

Quand vous êtes prêt, dites-moi et je vous guiderai étape par étape pour créer le premier package ! 🎓
