#!/bin/bash

# Script to perform multiple client tests for various vector sizes

# Number of times to call the service
num_calls=100

# Array of vector sizes to test
vector_sizes=(10 50 100 200 500 1000)

# Directory to save the output files
output_dir="results"
mkdir -p $output_dir

# Server command (assuming the server is run separately)
server_command="ros2 run ros2_rt_eval rt_srv_node"

# Start the server (assuming it's running in the background)
$server_command &

# Give the server some time to start
sleep 5

# Iterate over each vector size
for vector_size in "${vector_sizes[@]}"; do
    # Define the output file for the current vector size
    output_file="${output_dir}/latencies_vector_${vector_size}.csv"

    # Run the client command with the current vector size
    ros2 run ros2_rt_eval rt_cli_node $num_calls $vector_size $output_file

    echo "Completed test for vector size $vector_size. Results saved to $output_file."
done

# Optionally, stop the server after tests are done (if running in the background)
pkill -f rt_srv_node
