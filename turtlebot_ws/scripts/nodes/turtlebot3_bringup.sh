

function tmux_turtlebot_bringup
{
    # close previous session if open 
    tmux kill-session turtlebot_bringup
    sleep 2

    # clear new session and run bringup command
    tmux new-session -s turtlebot_bringup -n bringup
    tmux send-keys -t turtlebot_run:bringup "source_turtlebot_env" C-m
    tmux send-keys -t turtlebot_run:bringup "ros2 launch turtlebot3_bringup robot.launch.py" C-m
}

function tmux_turtlebot_rviz
{
    tmux kill-session turtlebot_rviz_tools
    sleep 2

    # clear new session and run bringup command
    tmux new-session -s turtlebot_rviz_tools -n rviz0  
    tmux send-keys -t turtlebot_rviz_tools:rviz0 "turtlebot_rviz" "Enter"
}

function turtlebot_rviz
{
    source_turtlebot_env
    ros2 launch turtlebot3_bringup rviz2.launch.py
}