# variables
turtlebot_repo_path="/home/$USER/git/lior-sc/turtlebot3_repo"
turtlebot_pc_ws_path="$turtlebot_repo_path/turtlebot_pc_ws"
turtlebot_pc_components_ws_path = "$turtlebot_repo_path/turtlebot_components_ws/"


# Functions

function add_source_wss_to_basrc
{
    echo "Do you want to source the components_ws in bashrc? (y/n)"
    read input

    # Check the value of the input
    if [ "$input" = "y" ]; then
    # Perform some action if the input is "yes"
        echo "" >> ~/.bashrc
        echo "## Source turtlebot3 pc workspaces" >> ~/.bashrc
        echo "source $turtlebot_pc_ws_path/scripts/turtlebot_pc_setup.sh" >> ~/.bashrc
        echo "source_turtlebot3_env" >> ~/.bashrc
    else
    # Perform some action if the input is not "yes"
    echo "Exiting."
    fi

}

# Main installation
echo "starting turtlebot PC package installations"
echo "make sure turtlebot3 packages are not installed in binary form"

echo "install ros foxy gazebo 11"
sudo apt-get install ros-foxy-gazebo-*

echo "install cartographer"
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros

echo "install NAV2"
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup


echo "Creating turtlebot3_components_ws and cloning packages"
cd $turtlebot_repo_path
mkdir -p turtlebot3_components_ws/src
cd $turtlebot_repo_path/turtlebot3_components_ws/src

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git --depth=1
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git


echo "build components ws"
cd /home/$USER/git/lior-sc/turtlebot3_components_ws/
colcon build --symlink-install
source $turtlebot_pc_components_ws_path/install/local_setup.bash

add_source_wss_to_basrc

echo "finished!"
