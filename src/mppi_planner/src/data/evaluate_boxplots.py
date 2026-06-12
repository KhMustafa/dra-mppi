import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
import json 
import pandas as pd

########################################### 4 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs.txt")
json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_4obs")  # Replace with your actual absolute path

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_4 = np.array([float(line.strip()) for line in lines[72418:72517]])
# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_4= data["duration"]

########################################### 8 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs.txt")
json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_8obs")  # Replace with your actual absolute path

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_8 = np.array([float(line.strip()) for line in lines[93099:93190]])
# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_8= data["duration"]

# Data for three different experiments (4, 8, 12 obstacles)
dur_data_mpc_all = [dur_data_mpc_4, dur_data_mpc_8]  # MPC
dur_data_mppi_all = [dur_data_mppi_4, dur_data_mppi_8]  # MPPI
experiments = [4, 8]  # Y-axis labels

# Define colors to match the original image
colors = {
    "T-MPC (ours)": "#9ACD32",  # Yellow-green
    "T-MPC++ (ours)": "#FFD700"  # Gold
}

# Prepare data for plotting
data = []
methods = []
experiment_idx = []

for i, exp in enumerate(experiments):
    data.extend(dur_data_mpc_all[i])
    data.extend(dur_data_mppi_all[i])
    methods.extend(["T-MPC (ours)"] * len(dur_data_mpc_all[i]) + ["T-MPC++ (ours)"] * len(dur_data_mppi_all[i]))
    experiment_idx.extend([exp] * (len(dur_data_mpc_all[i]) + len(dur_data_mppi_all[i])))

# Create the plot
plt.figure(figsize=(8, 6))
sns.boxplot(x=data, y=experiment_idx, hue=methods, palette=colors, showfliers=False)
sns.stripplot(x=data, y=experiment_idx, hue=methods, palette=colors, jitter=True, dodge=True, alpha=0.6, size=4)

# Formatting
plt.axvline(12, linestyle="dashed", color="black")  # Vertical reference line at 12s
plt.xlabel("Task Duration (s)")
plt.ylabel("Experiment")
plt.legend(title=None, loc="upper right", frameon=True)
plt.grid(axis="x", linestyle="--", alpha=0.7)

plt.show()

########################################### 12 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs.txt")
json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_12obs")  # Replace with your actual absolute path

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_12 = np.array([float(line.strip()) for line in lines[70483:70582]])
# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_12 = data["duration"]


# Data for three different experiments (4, 8, 12 obstacles)
dur_data_mpc_all = [dur_data_mpc_4, dur_data_mpc_8, dur_data_mpc_12]  # MPC
dur_data_mppi_all = [dur_data_mppi_4, dur_data_mppi_8, dur_data_mppi_12]  # MPPI
experiments = [4, 8, 12]  # Y-axis labels

# Define colors to match the original image
colors = {
    "MPC": "#9ACD32",  # Yellow-green
    "MPPI": "#FFD700"  # Gold
}

# Prepare data for plotting
data = []
methods = []
experiment_idx = []

for i, exp in enumerate(experiments):
    data.extend(dur_data_mpc_all[i])
    data.extend(dur_data_mppi_all[i])
    methods.extend(["MPC"] * len(dur_data_mpc_all[i]) + ["MPPI"] * len(dur_data_mppi_all[i]))
    experiment_idx.extend([exp] * (len(dur_data_mpc_all[i]) + len(dur_data_mppi_all[i])))




'''

# Create a combined dataset for plotting
data = {
    "MPC Task Duration (s)": dur_data_mpc,
    "MPPI Task Duration (s)": dur_data_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC Task Duration (s)") for val in dur_data_mpc] + \
            [(val, "MPPI Task Duration (s)") for val in dur_data_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)

# Add vertical median lines
plt.axvline(np.median(dur_data_mpc), linestyle="--", color="blue", label="MPC Duration Median")
plt.axvline(np.median(dur_data_mppi), linestyle="--", color="red", label="MPPI Duration Median")

# Format plot
plt.xlabel("Values")
plt.title("Comparison of Task Duration")
plt.legend()
plt.show()
'''