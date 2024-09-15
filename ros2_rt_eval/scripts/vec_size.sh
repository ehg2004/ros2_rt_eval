#!/bin/bash

# Usage: ./run_client_multiple_times.sh <number_of_executions> <vector_size> <directory>
# Example: ./run_client_multiple_times.sh 10 100 /path/to/csv_dir

source ~/.bashrc
source /opt/ros/foxy/setup.bash && echo 'Foxy is active!'
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash;
source ~/ros2_ws2/install/setup.bash

if [ "$#" -ne 3 ]; then
    echo "Usage: $0  <vector_size> <directory> <number_of_executions>"
    exit 1
fi

# Get the number of times to run the client, vector size, and directory
VECTOR_SIZE=$1
DIR=$2
NEXE=$3

# Make sure the directory exists
mkdir -p "$DIR"

# Server command (assuming the server is run separately)
#server_command="ros2 run ros2_rt_eval rt_srv_node  1"

# Start the server (assuming it's running in the background)
#$server_command &

# Give the server some time to start
sleep 1




# Run the client node N times with varying Client_ID
for ((i=1; i<=$VECTOR_SIZE; i*=10))
do

    # Execute the ROS 2 client node, passing the Client_ID, vector_size, and directory
    ros2 run ros2_rt_eval rt_cli_node  $NEXE $i $i 1 $DIR &
    # <num_calls> <vector_size> <client_id> <dir>

    # Check if the client node executed successfully
     if [ $? -ne 0 ]; then
        echo "Client execution failed for Client_ID=$i. Exiting."
        exit 1
     fi
    sleep 5
done
