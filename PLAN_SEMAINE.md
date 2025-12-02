# 📅 Plan pour la semaine prochaine

## 🎯 Objectif final
Créer un robot qui cherche un objet (ex: chien) de manière autonome et s'arrête quand il le trouve.

---

## 📦 Package 1 : `object_search_navigation` (Vous + JP + Hadil)

### Responsabilités
1. **Naviguer** dans l'environnement (déplacement autonome)
2. **Publier** le flux de la caméra sur un topic ROS2
3. **Éviter** les obstacles (utiliser le Lidar `/scan`)

### Topics utilisés
- **Subscribe** (écoute) :
  - `/camera/image_raw` : Images de la caméra
  - `/scan` : Données du Lidar (obstacles)
  - `/search/status` : Statut du détecteur (SEARCHING/STOP)
  
- **Publish** (publie) :
  - `/cmd_vel` : Commandes de mouvement
  - `/search/camera_feed` : Flux caméra pour le détecteur

### Stratégie de navigation simple
- Avancer tout droit
- Si obstacle détecté → tourner
- Si "STOP" reçu → arrêter le robot

---

## 🔍 Package 2 : `object_detector`

### Responsabilités
1. **S'abonner** au flux caméra
2. **Analyser** les images pour détecter l'objet cible
3. **Publier** "STOP" quand l'objet est trouvé

### Topics utilisés
- **Subscribe** :
  - `/search/camera_feed` : Images à analyser
  
- **Publish** :
  - `/search/status` : "SEARCHING" ou "STOP"

### Méthode de détection (à choisir)
- **Option 1 (simple)** : Détection par couleur (OpenCV)
- **Option 2 (avancé)** : YOLO pour détecter "chien", "bouteille", etc.

---

## 🗺️ Packages existants (déjà fournis)

### Cartographer
- **Rôle** : Créer une carte de l'environnement
- **Utilité** : Savoir où sont les obstacles
- **Topic** : Publie la carte sur `/map`

### Nav2
- **Rôle** : Localiser le robot sur la carte
- **Utilité** : Connaître la position exacte du robot
- **Topic** : Publie la position sur `/amcl_pose`

### Teleop
- **Rôle** : Contrôle manuel
- **Utilité** : Tester et débugger
- ✅ Déjà testé !

---

## 📝 Plan d'action étape par étape

### Étape 1 : Créer le package de navigation (2-3h)
```bash
cd ~/Autonomous_intelligent_systems/src
ros2 pkg create --build-type ament_python object_search_navigation \
  --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
```

### Étape 2 : Coder le node de navigation (3-4h)
Créer `object_search_navigation/navigation_node.py` avec :
- Subscriber caméra
- Subscriber Lidar (obstacles)
- Subscriber status (STOP/SEARCHING)
- Publisher cmd_vel (mouvement)
- Publisher camera_feed (republier caméra)

### Étape 3 : Créer le package détecteur (2-3h)
```bash
cd ~/Autonomous_intelligent_systems/src
ros2 pkg create --build-type ament_python object_detector \
  --dependencies rclpy sensor_msgs std_msgs cv_bridge opencv-python
```

### Étape 4 : Coder le détecteur (3-4h)
Créer `object_detector/detector_node.py` avec :
- Subscriber camera_feed
- Publisher status
- Logique de détection (couleur ou YOLO)

### Étape 5 : Tester séparément (1-2h)
- Tester navigation seule
- Tester détection seule

### Étape 6 : Intégration (1-2h)
- Lancer les deux packages ensemble
- Vérifier que le robot s'arrête quand objet trouvé

---

## ⏱️ Estimation totale : 12-18 heures de travail

Si vous êtes 3 personnes (vous + JP + Hadil) :
- **4-6 heures par personne**
- Répartition suggérée :
  - **Personne 1** : Navigation + évitement obstacles
  - **Personne 2** : Streaming caméra + intégration
  - **Personne 3** : Détection d'objets

---

## 🚀 Première action MAINTENANT

**Question** : Voulez-vous que je vous guide pour créer le **Package 1** (navigation) maintenant ?

Si oui, on va :
1. Créer la structure du package (5 min)
2. Écrire le code du node de navigation (30 min)
3. Tester en simulation (10 min)

**Répondez simplement "oui" pour commencer, ou posez vos questions !** 😊
