#!/bin/bash

BASE_DIR="/home/amit-singh/Downloads/qudacopter/autonomous-qudacopter"

# Array to hold terminal process groups
declare -a TERMINAL_PGIDS=()

# Function to launch a terminal and store its process group ID
launch_terminal() {
    local cmd="$1"
    gnome-terminal -- bash -c "$cmd; exec bash" &
    local pid=$!
    # Get the process group ID of the spawned terminal
    local pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')
    TERMINAL_PGIDS+=($pgid)
    echo "Launched terminal with PGID $pgid"
}

rm -f ${BASE_DIR}/log/term1.log
rm -f ${BASE_DIR}/log/term2.log
rm -f ${BASE_DIR}/log/term3.log

# Terminal 1
launch_terminal "echo 'Terminal 1: Running simulator'; cd /home/amit-singh/Downloads/qudacopter/jMAVSim/PX4-Autopilot; export PX4_GZ_MODEL_POSE="5,5,0,0,0,0"; make px4_sitl gz_x500_lidar_2d_home | tee ${BASE_DIR}/log/term1.log"

# launch_terminal "echo 'Terminal 1: Running simulator'; cd /home/amit-singh/Downloads/qudacopter/jMAVSim/PX4-Autopilot; make px4_sitl gz_x500 | tee ${BASE_DIR}/log/term1.log"

# Wait until Terminal 1 prints a specific string
echo "Waiting for Terminal 1 output..."
while true; do
    if tail -n 100 ${BASE_DIR}/log/term1.log | grep -F "INFO  [px4] Startup script returned successfully"; then
        echo "Condition met: launching Terminal 2"
        launch_terminal "echo 'Terminal 2 running'; cd ${BASE_DIR}/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; MicroXRCEAgent udp4 -p 8888 -v | tee ${BASE_DIR}/log/term2.log"
        break
    fi
    sleep 1
done

# Wait until Terminal 2 prints a specific string
echo "Waiting for Terminal 2 output..."
while true; do
    if tail -n 100 ${BASE_DIR}/log/term1.log | grep -F "INFO  [uxrce_dds_client] time sync converged"; then
        echo "Condition met: launching Terminal 3"
        launch_terminal "echo 'Terminal 3 running'; cd ${BASE_DIR}/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; . ../simulation_start.sh | tee ${BASE_DIR}/log/term3.log"
        break
    fi
    sleep 1
done

# Prompt to kill all terminals
read -p "Press 'e' to kill all launched terminals: " input
if [[ "$input" == "e" ]]; then
    echo "Killing all launched terminals..."
    for pgid in "${TERMINAL_PGIDS[@]}"; do
        echo "Killing process group $pgid"
        ps -ef | grep -E "gz|ros2|8888" | grep -v grep | awk '{print $2}' | xargs kill -${2:-'TERM'} 2>/dev/null
        kill -9 -"$pgid" 2>/dev/null
    done
    ps -ef | grep -E "gz|ros2|8888" | grep -v grep | awk '{print $2}' | xargs kill -${2:-'TERM'}
    echo "All spawned terminals killed."
fi
