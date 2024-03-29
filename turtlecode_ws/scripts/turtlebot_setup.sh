
###################### variables ######################
turtlebot_repo_path="/home/$USER/git/lior-sc/turtlebot3_repo"
turtlebot_ws_path="$turtlebot_repo_path/turtlebot_ws"
turtlebot_components_ws_path="$turtlebot_repo_path/turtlebot_components_ws"


###################### functions ######################
function foxy
{
    source /opt/ros/foxy/setup.bash
    echo "source foxy"
}

function source_node_scripts
{
    source $turtlebot_ws_path/scripts/nodes/turtlebot3_bringup.sh
}

function source_turtlebot_env
{
    echo "source foxy"
    source /opt/ros/foxy/setup.bash
    echo "source turtlebot components"
    source $turtlebot_components_ws_path/install/local_setup.bash
    echo "source turtlebot_ws"
    source $turtlebot_ws_path/install/local_setup.bash

    source_node_scripts
    
    # echo "change ROS_DOMAIN and TURTLEBOT_MODEL"
    export ROS_DOMAIN_ID=30 
    export TURTLEBOT3_MODEL=burger
    export LDS_MODEL=LDS-01
    
    # echo "turtlebot3 env has been sourced"
}

function ssh_turtlebot
{
    echo "SSH turtlebot3"
    echo "Turtlebot3 password is: turtlebot"
    ssh ubuntu@192.168.1.202
}

function build_turtlebot_ws
{
    source_turtlebot_env
    echo ""
    # Store present working directory
    pwd=$PWD
    #go to workspace path
    cd $turtlebot_ws_path
    # build workspace
    colcon build --symlink-install
    # source worksapce
    source $turtlebot_ws_path/install/local_setup.bash
    # go back to previous working directory
    cd $pwd
}

function build_turtlebot_ws_pkg 
{
    pkg=$1  #package to build

    source_turtlebot_env
    echo ""
    # Store present working directory
    pwd=$PWD
    #go to workspace path
    cd $turtlebot_ws_path
    # build workspace
    colcon build --symlink-install --packages-up-to $pkg
    # source worksapce
    source $turtlebot_ws_path/install/local_setup.bash
    # go back to previous working directory
    cd $pwd
}

function build_turtlebot_components
{
    source_turtlebot_env
    echo ""
    current_path=$PWD
    cd $turtlebot_components_ws_path
    colcon build --symlink-install
    source $turtlebot_components_ws_path/install/local_setup.bash

    cd $current_path
}

function source_bashrc
{
    source /home/$USER/.bashrc
    echo "source bashrc"
}


