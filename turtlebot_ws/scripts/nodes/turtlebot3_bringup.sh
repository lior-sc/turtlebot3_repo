

function tmux_turtlebot_bringup
{
    # close previous session if open 
    tmux kill-session turtlebot_bringup
    sleep 5

    # clear new session and run bringup command
    tmux new-session -s turtlebot_bringup -n bringup
    tmux send-keys -t turtlebot_run:bringup "ros2 launch turtlebot3_bringup robot.launch.py" C-m
}