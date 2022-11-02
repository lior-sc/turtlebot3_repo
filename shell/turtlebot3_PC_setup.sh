
###################### variables ######################
turtlebot3_repo_path="/home/$USER/git/lior-sc/turtlebot3_repo"
turtlebot3_ws_path="$turtlebot3_repo_path/turtlebot3_ws"


###################### functions ######################
source_turtlebot3_env(){
    source /opt/ros/foxy/setup.bash
    source $turtlebot3_ws_path/install/local_setup.bash
    cd $turtlebot3_repo_path
    
    export ROS_DOMAIN_ID=30 
    export TURTLEBOT_MODEL=burger
    
    echo "turtlebot3 env sourced"
}

