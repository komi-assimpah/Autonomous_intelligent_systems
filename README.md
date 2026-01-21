# Système Robotique Autonome de Recherche d'Objet (Autonomous Intelligent Systems)

## Présentation du Projet
Ce projet consiste à développer un **robot mobile** capable d'explorer un environnement pour trouver un objet spécifique (ex: chien, chat, bouteille, etc.), **de manière totalement autonome**.

### Le Concept
Le robot combine **Exploration Autonome** et **Intelligence Artificielle**.
1.  Il **explore** la pièce en évitant les murs.
2.  Il **détecte** visuellement sa cible.
3.  Il **s'arrête**, localise l'objet sur la carte et renvoie les coordonnées de l'objet trouvé dans le repère de la map.

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

### Logiciels prérequis

### Install a Virtual Machine (VM)
- On windows : VirtualBox (https://www.virtualbox.org)
- On MacOSX : UTM (https://mac.getutm.app)

### Ubuntu 22.04
-  https://releases.ubuntu.com/jammy/

! ARM image is required for Apple Silicon-based laptops,
- https://cdimage.ubuntu.com/releases/22.04/release/
- Only the server version is available… need to manually install GUI…

```bash
$> sudo apt update & sudo apt upgrade & sudo apt install ubuntu-desktop
```
the reboot your computer

### ROS2 Humble

- install ROS2 specific packages

```bash
$> sudo apt install ros-humble-ros-gz

$> sudo apt install ros-humble-nav2-map-server
$> sudo apt install ros-humble-cartographer
$> sudo apt install ros-humble-cartographer-ros
$> sudo apt install ros-humble-navigation2
$> sudo apt install ros-humble-nav2-bringup
$> sudo apt install ros-humble-turtlebot3-msgs
$> sudo apt install ros-humble-xacro
```

- install colcon, the ROS2 package compiler
```bash
sudo apt install python3-colcon-common-extensions python3-argcomplete libboost-system-dev
```

- And cyclone DDS
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Then (because ??):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

source .bashrc

printenv | grep -i ROS (???)
```

Verify if there is any missing package dependencies and install them with :

```bash
sudo apt install build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

Verify ROS2 installation

```bash
$> ros2  
usage: ros2 [-h] [--use-python-default-buffering] Call 'ros2 <command> -h' for more detailed usage. …  
ros2 is an extensible command-line tool for ROS 2.  
…  
```

### Gazebo

Ce projet utilise **Gazebo Ignition Fortress** (compatible avec ROS2 Humble).

```bash
# Installation de Gazebo Ignition Fortress
sudo apt install ros-humble-ros-gz

# Vérifier l'installation
ign gazebo --version
# Devrait afficher : Gazebo Sim, version 6.x.x
```

> **Note** : Ne pas confondre avec `gazebo` (Gazebo Classic) ou `gz` (Gazebo Garden+).
> Ce projet utilise `ign gazebo` (Ignition Fortress).

Documentation officielle : https://gazebosim.org/docs/fortress/install_ubuntu

---

## Installation pour la Simulation (Gazebo)

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

# 2. Créer un environnement virtuel Python
python3 -m venv venv
source venv/bin/activate

# 3. Récupérer les sous-repos (TurtleBot3, Dynamixel...)
vcs import < dependencies.repos

# 4. Installer les dépendances ROS
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 5. Installer les libs Python (YOLO, etc.)
pip install -r requirements.txt
```

### 3. Compiler
```bash
colcon build
# ... (this may take a while) ...

source install/setup.bash
```

### 4. Configurer l'Environnement
A chaque nouveau terminal ouvert avant de lancer le robot, lancez ces commandes:
```bash
source venv/bin/activate
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source install/setup.bash
```

ou ajouter un alias dans votre `~/.bashrc` :
```bash
alias robot_env='source ~/Autonomous_intelligent_systems/venv/bin/activate && export TURTLEBOT3_MODEL=burger && source /opt/ros/humble/setup.bash && source ~/Autonomous_intelligent_systems/install/setup.bash'
```

### 5. Lancer la Simulation
Lancer Gazebo, RViz et toute l'intelligence avec la commande :
```bash
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true
```

Par défaut, un chien est recherché. Pour spécifier l'objet à chercher, assurez-vous d'avoir complètement arrêté la simulation précédente, ensuite lancez :

```bash
# Pour chercher un objet spécifique
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='nom_de_la_classe_cible'

# Exemples :
# Pour chercher un chien
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='dog'

# Pour chercher un chat
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='cat'
```

Si cet objet n'est pas supporté par le modèle de recherche, vous aurez une erreur dans le terminal avec la liste de toutes les classes supportées par le modèle.

---

## Installation pour le Robot Réel (TurtleBot3)

> **Note** : Cette section est en cours de validation.

### Paquets supplémentaires requis
Ces paquets ne sont **pas nécessaires** pour la simulation Gazebo, mais requis pour un TurtleBot3 physique :

```bash
sudo apt install ros-humble-hls-lfcd-lds-driver

sudo apt install ros-humble-dynamixel-sdk

sudo apt install libudev-dev
```

### Lancer sur le Robot Réel
```bash
ros2 launch robot_orchestrator orchestrator.launch.py sim:=false
```

---


