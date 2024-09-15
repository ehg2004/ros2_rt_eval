#!/bin/bash

# Usage: ./run_client_multiple_times.sh <number_of_executions> <vector_size> <directory>
# Example: ./run_client_multiple_times.sh 10 100 /path/to/csv_dir

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <number_of_clients> <vector_size> <directory> <number_of_executions>"
    exit 1
fi

# Get the number of times to run the client, vector size, and directory
N=$1
VECTOR_SIZE=$2
DIR=$3
NEXE=$4

# Make sure the directory exists
mkdir -p "$DIR"

# Server command (assuming the server is run separately)
server_command="ros2 run ros2_rt_eval rt_srv_node 1"

# Start the server (assuming it's running in the background)
#$server_command &

# Give the server some time to start
sleep 5

# Run the client node N times with varying Client_ID
for ((i=1; i<=N; i*=10))
do
    echo "Running client with Client_ID=$i"
    
    # Execute the ROS 2 client node, passing the Client_ID, vector_size, and directory
    ros2 run ros2_rt_eval rt_cli_node  $NEXE $VECTOR_SIZE $i 1 $DIR
    # <num_calls> <vector_size> <client_id> <dir>

    # Check if the client node executed successfully
    if [ $? -ne 0 ]; then
        echo "Client execution failed for Client_ID=$i. Exiting."
        exit 1
    fi
    sleep 1
done

echo "Client executed $N times successfully!"
