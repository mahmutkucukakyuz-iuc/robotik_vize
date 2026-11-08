#!/usr/bin/env bash
# rk6 vize projesi - WSL2 Ubuntu 26.04 icin ROS 2 Lyrical + Gazebo Jetty kurulumu ve derleme.
# Calistirma (Ubuntu terminalinde, normal kullanici olarak; sudo sifresini bir kez sorar):
#   bash "<depo_klasoru>/kurulum_wsl.sh"
#   (ornek: bash "/mnt/c/Users/<kullanici>/.../vize_hazirlik/kurulum_wsl.sh" ya da bash ~/rt_ws/src/rk6/kurulum_wsl.sh)
# Paketler betigin bulundugu deponun ros2_ws/src klasorunden ~/rt_ws/src altina kopyalanip derlenir.
set -e

PROJE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros2_ws/src"
WS="$HOME/rt_ws"

echo "== 1/6 Dil ayari ve temel araclar"
sudo apt update
sudo apt install -y locales software-properties-common curl rsync mesa-utils
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

echo "== 2/6 ROS 2 apt deposu"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt update
sudo apt upgrade -y

echo "== 3/6 ROS 2 Lyrical + Gazebo Jetty (vendor paketleri) + proje bagimliliklari"
sudo apt install -y ros-lyrical-desktop ros-dev-tools \
  ros-lyrical-ros-gz ros-lyrical-gz-ros2-control \
  ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
  ros-lyrical-xacro ros-lyrical-robot-state-publisher \
  ros-lyrical-joint-state-publisher-gui ros-lyrical-rviz2 \
  ros-lyrical-cv-bridge ros-lyrical-rqt-image-view ros-lyrical-tf2-tools \
  python3-numpy python3-matplotlib python3-opencv

echo "== 4/6 ~/.bashrc ayarlari (ROS ortami + NVIDIA ekran karti)"
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
add_line 'source /opt/ros/lyrical/setup.bash'
add_line '[ -f ~/rt_ws/install/setup.bash ] && source ~/rt_ws/install/setup.bash'
# WSL varsayilan olarak yazilimsal cizim (llvmpipe) kullaniyor; d3d12 ile NVIDIA ekran karti secilir
add_line 'export GALLIUM_DRIVER=d3d12'
add_line 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA'
# WSLg'de Qt Wayland kullaniyor, Ogre (RViz/Gazebo) ise X11 istiyor: "Invalid parentWindowHandle" hatasini onler
add_line 'export QT_QPA_PLATFORM=xcb'

echo "== 5/6 Proje dosyalarini $WS/src altina kopyala"
mkdir -p "$WS/src"
case "$PROJE/" in
  "$WS/src/"*) echo "   depo zaten $WS/src icinde (git clone), kopyalama atlandi" ;;
  *) rsync -a --delete "$PROJE/" "$WS/src/" ;;
esac

echo "== 6/6 Derleme"
set +u
source /opt/ros/lyrical/setup.bash
cd "$WS"
colcon build --symlink-install

echo
echo "Kurulum tamam. Yeni bir Ubuntu terminali acin (veya: source ~/.bashrc) ve deneyin:"
echo "  ros2 launch rk6_description display.launch.py"
echo "  ros2 launch rk6_description gazebo.launch.py rviz:=true"
