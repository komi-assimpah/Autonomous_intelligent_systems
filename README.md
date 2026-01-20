# Système Robotique Autonome de Recherche d'Objet (Autonomous Intelligent Systems)

## Présentation du Projet
Ce projet consiste à développer un **robot mobile** capable d'explorer un environnement pour trouver un objet spécifique (ex: chien, chat, bouteille, etc.), **de manière totalement autonome**.

### Le Concept
Le robot combine **Exploration Autonome** et **Intelligence Artificielle**.
1.  Il **explore** la pièce en évitant les murs.
2.  Il **détecte** visuellement sa cible.
3.  Il **s'arrête** et localise l'objet sur la carte et renvoie les coordonnées de l'objet trouvé dns le repère de la map.

---


## Architecture du Système
Le système repose sur 3 Packages ROS 2 :

- **`ia_package`** : Détection d'objets (YOLOv11) et positionnement 3D dans les différentes repères (CAMERA et MAP).
- **`robot_orchestrator`** : Machine à états qui pilote la mission globale.
- **`object_search_navigation`** : Algorithme d'exploration et d'évitement d'obstacles.

### Ordonnancement de la Mission
Le diagramme ci-dessous illustre le déroulement complet d'une mission, de l'initialisation à l'arrêt, et les interactions entre les 3 packages et l'environnement :

![Séquence de la mission](docs/architecture.png)


---

## Installation & Démarrage

### 1. Cloner le Projet
```bash
git clone https://github.com/komi-assimpah/Autonomous_intelligent_systems.git
cd Autonomous_intelligent_systems
```

### 2. Installer les Dépendances
On utilise un script pour tout récupérer d'un coup.

```bash
# 1. Outils de build
sudo apt install python3-vcstool python3-colcon-common-extensions -y

# 2. Récupérer les sous-repos (TurtleBot3, Dynamixel...)
vcs import < dependencies.repos

# 3. Installer les dépendances ROS
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 4. Installer les libs Python (YOLO, etc.)
pip3 install -r requirements.txt
```

### 3. Compiler
```bash
colcon build
source install/setup.bash
```

### 4. Configurer l'Environnement
a chaque nouveau terminal ouvert avant de lancer le robot, lancez ces commande (ou créez un alias) :
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Lancer la démo

### En Simulation (PC)
Lancer Gazebo, RViz et toute l'intelligence avec la commande
```bash
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true
```
Par défaut, un chien est recherché. Pour spécifier l'objet à chercher, lancez :

```bash
# Pour chercher un chien
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='dog'

# Pour chercher un chat
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='cat'
```

Si cet objet n'est pas supporté par le modèle de recherche, vous aurez une erreur dans le terminal avec la liste de toutes les classes supportées par le modèle et donc recherchables.

<!--
### Robot Réel
*Note : Section en attente de test matériel.*

```bash
ros2 launch robot_orchestrator orchestrator.launch.py sim:=false
```
-->


---



---


