import os
import pandas as pd
import matplotlib.pyplot as plt

# Directory containing the results files
output_dir = "results"

# Get the list of CSV files
csv_files = [f for f in os.listdir(output_dir) if f.endswith(".csv")]

# Determine the number of subplots needed
num_files = len(csv_files)

# Create subplots
fig, axs = plt.subplots(num_files, 1, figsize=(10, num_files * 4))

# If only one file, axs will not be a list, so we make it a list
if num_files == 1:
    axs = [axs]

# Iterate over each CSV file and corresponding subplot
for ax, filename in zip(axs, csv_files):
    # Construct the full path to the file
    file_path = os.path.join(output_dir, filename)
    
    # Read the CSV file into a pandas DataFrame
    data = pd.read_csv(file_path)
    
    # Extract vector size from the filename
    vector_size = filename.split('_')[-1].replace('.csv', '')
    
    # Plot the data in the current subplot
    ax.plot(data['Iteration'].values, data['Latency (microseconds)'].values, label=f'Vector Size {vector_size}')
    
    # Customize the subplot
    ax.set_title(f"Latency vs. Iteration for Vector Size {vector_size}")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Latency (microseconds)")
    ax.legend()
    ax.grid(True)

# Adjust layout to prevent overlapping
plt.tight_layout()

# Save the plot as an image file
plt.savefig("latency_subplots.png")

# Display the plot
plt.show()
