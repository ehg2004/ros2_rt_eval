import csv
import matplotlib.pyplot as plt
import os

def merge_csv_files(client_file, server_file, output_file):
    with open(client_file, 'r') as client_csv, open(server_file, 'r') as server_csv, open(output_file, 'w', newline='') as output_csv:
        client_reader = csv.reader(client_csv)
        server_reader = csv.reader(server_csv)
        output_writer = csv.writer(output_csv)

        # Write header for the output file
        output_writer.writerow(['Client_ID', 't1', 't2', 't3', 't4', 't5', 't6'])

        # Skip the headers in the input files
        next(client_reader)
        next(server_reader)

        # Iterate through both CSV files row by row
        for client_row, server_row in zip(client_reader, server_reader):
            # Extract values from client and server rows
            client_id = client_row[0]
            t1 = float(client_row[1])
            t2 = float(client_row[2])
            t3 = float(server_row[1])
            t4 = float(server_row[2])

            # Calculate t5 and t6
            t5 = t3 - t1
            t6 = t4 - t2 ## t2- t4 (???)

            # Merge the rows and write them to the output file
            output_writer.writerow([client_id, t1, t2, t3, t4, t5, t6])

        print(f"Successfully merged CSV files and saved to {output_file}")

def plot_t5_t6(output_file, output_dir):
    client_data = {}

    # Read the merged CSV and group data by client ID
    with open(output_file, 'r') as ofile:
        reader = csv.DictReader(ofile)

        for row in reader:
            client_id = row['Client_ID']
            t5 = float(row['t5'])
            t6 = float(row['t6'])

            if client_id not in client_data:
                client_data[client_id] = {'t5': [], 't6': []}

            client_data[client_id]['t5'].append(t5)
            client_data[client_id]['t6'].append(t6)

    # Plot t5 and t6 for each client
    for client_id, times in client_data.items():
        plt.figure(figsize=(8, 5))

        plt.plot(times['t5'], label='t5 (t3 - t1)', marker='o')
        plt.plot(times['t6'], label='t6 (t2 - t4)', marker='o')

        plt.xlabel('Iteration')
        plt.ylabel('Time Difference (s)')
        plt.title(f'Client ID: {client_id} - Plot of t5 and t6')
        plt.legend()
        plt.grid(True)

        # Save the plot as an image file in the output directory
        plot_filename = os.path.join(output_dir, f'{client_id}_plot.png')
        plt.savefig(plot_filename)
        plt.close()


if __name__ == "__main__":
    # Path to the client and server CSV files
    client_file = './results/24client.csv'  # Change this to your actual file
    server_file = './results/24server.csv'  # Change this to your actual file
    output_file = './results/24output.csv'  # Change this to your actual output file

    output_dir = './plots'

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    for i in range (1,2):
        client_file = './results/'+str(i)+'client.csv'  # Change this to your actual file
        server_file = './results/'+str(i)+'server.csv'  # Change this to your actual file
        output_file = './results/'+str(i)+'output.csv'  # Change this to your actual output file
        # Call the function
        merge_csv_files(client_file, server_file, output_file)
        plot_t5_t6(output_file,output_dir)

    

    


