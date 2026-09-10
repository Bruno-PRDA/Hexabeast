# Installer la chaîne de simulation : WSL2 + ROS 2 Jazzy + Gazebo Harmonic

Gazebo est un outil Linux. Sur Windows 11 il tourne dans WSL2, avec fenêtres
et accélération GPU via WSLg (intégré). La machine a ce qu'il faut :
virtualisation active, RTX 2000 Ada, 32 Go de RAM, 677 Go libres.

## 1. WSL — droits administrateur requis

Il faut un compte administrateur (sur un poste d'entreprise, souvent l'IT).
Dans un PowerShell **ouvert en administrateur** :

```
wsl --install -d Ubuntu-24.04
```

Redémarrer quand on le demande. Au premier lancement d'Ubuntu, choisir un nom
d'utilisateur et un mot de passe Linux. Vérifier ensuite depuis un PowerShell
normal :

```
wsl -l -v
```

`Ubuntu-24.04` doit apparaître en `VERSION 2`.

## 2. ROS 2 Jazzy + Gazebo Harmonic — dans Ubuntu

Le duo LTS (support jusqu'en 2029). Dans le terminal Ubuntu :

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

`ros-jazzy-ros-gz` tire Gazebo Harmonic, la version appariée à Jazzy.
Compter ~10 Go et une vingtaine de minutes.

## 3. Le workspace

Compiler sur `/mnt/c/...` est lent (système de fichiers traversant). On clone
le dépôt dans le système de fichiers Linux :

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone /mnt/c/Users/<toi>/Documents/Robot hexapod   # ou git clone https://github.com/Brun0zinh0/Hexabeast.git hexapod
cd ~/ros2_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Les paquets sont dans `hexapod/ros2_ws/src/` ; colcon les trouve en
parcourant `src/` récursivement.

## 4. Lancer

```bash
ros2 launch hexapod_description sim.launch.py
```

Gazebo s'ouvre sur le parcours (trottoir, colline), le robot apparaît et se
met debout. Dans un second terminal :

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` avance, `,` recule, `j`/`l` tournent, `J`/`L` marchent en crabe.

Pour vérifier le modèle seul, sans Gazebo, avec des curseurs par articulation :

```bash
ros2 launch hexapod_description display.launch.py
```

## 5. Regénérer la description après un changement de géométrie

Sur Windows comme sur Linux, Python suffit :

```bash
python tools/gen_urdf.py --check
```

`--check` fait passer la cinématique directe par la chaîne d'articulations de
l'URDF et la compare à l'IK : une erreur de signe d'axe est détectée avant
que Gazebo ne la voie.

## Points à vérifier au premier lancement

- `gz topic -l` doit lister les capteurs (`.../sensor/imu/imu`,
  `.../sensor/FL_foot/contact`, …). Si un pont reste muet, ajuster les noms
  dans `config/bridge.yaml`.
- `ros2 control list_controllers` doit montrer `leg_controller` et
  `joint_state_broadcaster` en `active`.
- Si le rendu Gazebo est noir ou plante sous WSLg, ajouter
  `--render-engine ogre` (moteur v1) aux `gz_args`.
