# 安裝篇

請先檢查 OS 是否符合要求
- x86_64
- GNU/Linux
- 6.8.0-87-generic
- Ubuntu 24.04.3 LTS
- RAM > 2GB
- ROM > 32GB

## 基本工作站套件
```bash
sudo apt update -y && sudo apt upgrade -y
sudo apt -y install htop net-tools git 
sudo apt -y install curl tree tmux nfs-common nano
```

## ROS2 安裝
詳細請參考 [ROS2 Jazzy 官網](https://docs.ros.org/en/jazzy/index.html)

**檢查套件並加入 ROS 庫**
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**安裝**
```bash
sudo apt update -y && sudo apt upgrade -y
sudo apt install ros-jazzy-ros-base
sudo apt install python3-colcon-common-extensions
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```


## Node.js 安裝
這是用來開發 wevUI 的程式
詳細請參考[Vue.js 官網](https://vuejs.org/guide/quick-start.html)

**安裝**
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install --lts
nvm use --lts
npm install -g @vue/cli 
npm install roslib
```

## Miniconda 安裝
詳細請參考 [Conda 官網](https://www.anaconda.com/docs/getting-started/miniconda/install#linux-x86)
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash ~/Miniconda3-latest-Linux-x86_64.sh
```