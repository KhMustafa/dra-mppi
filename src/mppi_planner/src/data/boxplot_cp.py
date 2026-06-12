import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import json 
import matplotlib.patches as mpatches
import random

########################################### 4 obs data ###########################################
json_path_mpc = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs.json")  # Replace with your actual absolute path
json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/4obs_joint_prob")

# Load JSON data from file
with open(json_path_mpc, "r") as file:
    data = json.load(file)

cp_mpc_4 = data["max_cp"]

mean_cp_mpc_4 = np.mean(cp_mpc_4)
std_cp_mpc_4 = np.std(cp_mpc_4)

# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

cp_mppi_4 = data["max_joint_mc"]
mean_cp_mppi_4 = np.mean(cp_mppi_4)
std_cp_mppi_4 = np.std(cp_mppi_4)
########################################### 8 obs data ###########################################
json_path_mpc = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs.json")  # Replace with your actual absolute path
json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/8obs_joint_prob")

# Load JSON data from file
with open(json_path_mpc, "r") as file:
    data = json.load(file)

cp_mpc_8 = data["max_cp"]
mean_cp_mpc_8 = np.mean(cp_mpc_8)
std_cp_mpc_8 = np.std(cp_mpc_8)
# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

cp_mppi_8 = data["max_joint_mc"]
mean_cp_mppi_8 = np.mean(cp_mppi_8)
std_cp_mppi_8 = np.std(cp_mppi_8)
########################################### 12 obs data ###########################################
json_path_mpc = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs")  # Replace with your actual absolute path
json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/16obs_joint_prob")

# Load JSON data from file
with open(json_path_mpc, "r") as file:
    data = json.load(file)

cp_mpc_12 = data["max_cp"]
mean_cp_mpc_12 = np.mean(cp_mpc_12)
std_cp_mpc_12 = np.std(cp_mpc_12)

# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

cp_mppi_12 = data["max_joint_mc"]
mean_cp_mppi_12 = np.mean(cp_mppi_12)
std_cp_mppi_12 = np.std(cp_mppi_12)

# Mapping experiment sets to their corresponding cp data
cp_data = {
    4: {"SH-MPC": cp_mpc_4,  "DRA-MPPI": cp_mppi_4},
    8: {"SH-MPC": cp_mpc_8, "DRA-MPPI": cp_mppi_8},
    12: {"SH-MPC": cp_mpc_12, "DRA-MPPI": cp_mppi_12},
}



# Define experiment sets
experiment_sets = [4, 8, 12]
methods = ["SH-MPC", "DRA-MPPI"]
colors = ["#58A399", "#6C7EB7"]   # "#58A399", "#E67E22", "#6C7EB7", "#E58BBD", "#B5CC55", "#F1C40F"
data = []

# Define spacing parameters
exp_spacing = 4  # Vertical gap between experiment sets
method_spacing = 1  # Spacing between methods within the same experiment

y_positions = {}
current_y = 0

y_labels = []  # Store labels for proper y-axis display
y_ticks = []  # Store corresponding tick positions

for exp_set in experiment_sets:
    for i, method in enumerate(methods):
        task_cp = cp_data[exp_set][method]
        label = f"Experiment {exp_set} - {method}"
        y_positions[label] = current_y  # Assign y position
        y_ticks.append(current_y)  # Tick position
        for td in task_cp:
            data.append((current_y, label, method, td))
        current_y += method_spacing  # Move to next method
        y_labels.append(f"{exp_set}")  # Label for y-axis

    
    current_y += exp_spacing  # Increase y-position for the next experiment set


# Create DataFrame
df = pd.DataFrame(data, columns=["Y-position", "Experiment", "Method", "CP"])

# Plot settings
plt.figure(figsize=(8, 6))
sns.set(style="whitegrid")

# Overlay stripplot for individual data points
sns.stripplot(
    x="CP", y="Experiment", data=df,
    palette=colors, jitter=0.05, size=6, alpha=0.6
)
# Create boxplot
ax = sns.boxplot(
    x="CP", y="Experiment", data=df,
    palette=colors, showcaps=False, fliersize=0, linewidth=1.8, width=0.175, boxprops={"facecolor": "none"}
)

# Manually create legend handles
legend_patches = [mpatches.Patch(color=colors[i], label=methods[i]) for i in range(len(methods))]

# Add legend to the plot
plt.legend(handles=legend_patches, title="Methods", loc="upper right", frameon=True, fontsize=16)
# Increase tick font size
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)


plt.xlabel("Maximum Collision Probability", fontsize=16)
plt.ylabel("Experiment", fontsize=16)
plt.savefig("boxplot_spacing.svg", format="svg", dpi=300)

plt.show()