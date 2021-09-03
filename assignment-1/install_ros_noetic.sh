# get ros ppa
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
# get updated keys based on https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

# install ros
sudo apt install ros-noetic-desktop-full

# setup your environment so that you can find ros commands
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# install extra deps
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
