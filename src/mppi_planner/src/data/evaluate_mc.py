import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import json 
import matplotlib.patches as mpatches
import random

########################################### 4 obs data ###########################################


json_path_mppi_4obs = os.path.abspath("/workspace/src/mppi_planner/src/data/4obs_joint_prob")  # Replace with your actual absolute path
json_path_mppi_8obs = os.path.abspath("/workspace/src/mppi_planner/src/data/8obs_joint_prob")  # Replace with your actual absolute path
json_path_mppi_12obs = os.path.abspath("/workspace/src/mppi_planner/src/data/16obs_joint_prob")  # Replace with your actual absolute path

json_path_mpc_4obs = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs.json")
json_path_mpc_8obs = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs.json")
json_path_mpc_12obs = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs")

# Load JSON data from file
with open(json_path_mppi_4obs, "r") as file:
    data_4obs = json.load(file)

with open(json_path_mppi_8obs, "r") as file:
    data_8obs = json.load(file)

with open(json_path_mppi_12obs, "r") as file:
    data_12obs = json.load(file)


cp_mppi_joint_mc_4obs = data_4obs["max_joint_mc"]
cp_mppi_joint_mc_8obs = data_8obs["max_joint_mc"]
cp_mppi_joint_mc_12obs = data_12obs["max_joint_mc"]

# Load JSON data from file
with open(json_path_mpc_4obs, "r") as file:
    data_4obs = json.load(file)

with open(json_path_mpc_8obs, "r") as file:
    data_8obs = json.load(file)

with open(json_path_mpc_12obs, "r") as file:
    data_12obs = json.load(file)

cp_mpc_joint_mc_4obs = data_4obs["max_cp"]
cp_mpc_joint_mc_8obs = data_8obs["max_cp"]
cp_mpc_joint_mc_12obs = data_12obs["max_cp"]




# Mapping experiment sets to their corresponding duration data
duration_data = {
    4: {"SH-MPC": cp_mpc_joint_mc_4obs, "DRA-MPPI": cp_mppi_joint_mc_4obs},
    8: {"SH-MPC": cp_mpc_joint_mc_8obs, "DRA-MPPI": cp_mppi_joint_mc_8obs},
    12: {"SH-MPC": cp_mpc_joint_mc_12obs, "DRA-MPPI": cp_mppi_joint_mc_12obs},
}



# Define experiment sets
experiment_sets = [4, 8, 12]
methods = ["SH-MPC", "DRA-MPPI"]
colors = ["#58A399", "#6C7EB7"]   # "#58A399", "#E67E22", "#6C7EB7", "#E58BBD", "#B5CC55", "#F1C40F"
data = []

# Define spacing parameters
exp_spacing = 1  # Vertical gap between experiment sets
method_spacing = 1  # Spacing between methods within the same experiment

y_positions = {}
current_y = 0

y_labels = []  # Store labels for proper y-axis display
y_ticks = []  # Store corresponding tick positions

for exp_set in experiment_sets:
    for i, method in enumerate(methods):
        task_durations = duration_data[exp_set][method]
        label = f"{exp_set}-Peds"
        y_positions[label] = current_y  # Assign y position
        y_ticks.append(current_y)  # Tick position
        for td in task_durations:
            data.append((current_y, label, method, td))
        current_y += method_spacing  # Move to next method
        y_labels.append(f"{exp_set}")  # Label for y-axis

    
    current_y += exp_spacing  # Increase y-position for the next experiment set


# Create DataFrame
df = pd.DataFrame(data, columns=["Y-position", "Experiment", "Method", "Task Duration"])

# Plot settings
plt.figure(figsize=(8, 10))
sns.set(style="whitegrid")

# Overlay stripplot for individual data points
sns.stripplot(
    x="Task Duration", y="Experiment", data=df,
    palette=colors, jitter=0.05, size=6, alpha=0.6
)
# Create boxplot
ax = sns.boxplot(
    x="Task Duration", y="Experiment", data=df,
    palette=colors, showcaps=False, fliersize=0, linewidth=1.8, width=0.15, boxprops={"facecolor": "none"}
)

# Manually create legend handles
legend_patches = [mpatches.Patch(color=colors[i], label=methods[i]) for i in range(len(methods))]

# Add legend to the plot
plt.legend(handles=legend_patches, title="Methods", loc="upper right", frameon=True, fontsize=16)
# Increase tick font size
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)

# Add vertical dashed line at 12 seconds
plt.axvline(x=0.05, linestyle="dashed", color="black", linewidth=1)

# Manually set y-axis labels to preserve stacking and gaps
#ax.set_yticks(y_ticks)
#ax.set_yticklabels(y_labels)

# Adjust legend position
# plt.legend(title="", frameon=True, loc="upper right")

# Labels and layout
plt.xlabel("Maximum Collision Probability", fontsize=16)
plt.ylabel("Experiment", fontsize=16)
plt.savefig("boxplot.svg", format="svg", dpi=300)

plt.show()