import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import json 
import matplotlib.patches as mpatches

########################################### 4 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs.txt")
file_path_05 = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs_05.txt")

json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/4obs_joint_prob")  # Replace with your actual absolute path
json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_4obs")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_4 = np.array([float(line.strip()) for line in lines[72418:72517]])

# Read the file
with open(file_path_05, "r") as file:
    lines = file.readlines()

dur_data_mpc_05_4 = np.array([float(line.strip()) for line in lines[96577:96676]])

# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_4= np.array(data["duration"])

# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi_4= np.array(data["duration"])

########################################### 8 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs.txt")
file_path_05 = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs_05.txt")

json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/8obs_joint_prob")  # Replace with your actual absolute path
json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_8obs")

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_8 = np.array([float(line.strip()) for line in lines[93099:93190]])

# Read the file
with open(file_path_05, "r") as file:
    lines = file.readlines()

dur_data_mpc_05_8 = np.array([float(line.strip()) for line in lines[99059:99155]])

# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_8= np.array(data["duration"])

# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi_8= np.array(data["duration"])

########################################### 12 obs data ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs.txt")
file_path_05 = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs_05.txt")

json_path_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/16obs_joint_prob")  # Replace with your actual absolute path
json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_12obs")

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

dur_data_mpc_12 = np.array([float(line.strip()) for line in lines[70483:70582]])

# Read the file
with open(file_path_05, "r") as file:
    lines = file.readlines()

dur_data_mpc_05_12 = np.array([float(line.strip()) for line in lines[104249:104345]])

# Load JSON data from file
with open(json_path_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi_12 = np.array(data["duration"])

# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi_12= np.array(data["duration"])

# Mapping experiment sets to their corresponding duration data
duration_data = {
    4: {"SH-MPC": dur_data_mpc_4, "Frenet Planner": dur_data_mpc_05_4, "DRA-MPPI": dur_data_mppi_4, "Vanilla MPPI": dur_data_vanilla_mppi_4},
    8: {"SH-MPC": dur_data_mpc_8, "Frenet Planner": dur_data_mpc_05_8,"DRA-MPPI": dur_data_mppi_8, "Vanilla MPPI": dur_data_vanilla_mppi_8},
    12: {"SH-MPC": dur_data_mpc_12, "Frenet Planner": dur_data_mpc_05_12,"DRA-MPPI": dur_data_mppi_12, "Vanilla MPPI": dur_data_vanilla_mppi_12},
}



# Set random seed for reproducibility
np.random.seed(42)

# Define experiment sets
experiment_sets = [4, 8, 12]
methods = ["SH-MPC", "Frenet Planner", "DRA-MPPI", "Vanilla MPPI"]
colors = ["#58A399", "#E67E22", "#6C7EB7", "#E58BBD"]   #  "#E58BBD", "#B5CC55", "#F1C40F"
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
        task_durations = duration_data[exp_set][method]
        label = f"Experiment {exp_set} - {method}"
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
    palette=colors, jitter=True, size=6, alpha=0.6
)
# Create boxplot
ax = sns.boxplot(
    x="Task Duration", y="Experiment", data=df,
    palette=colors, showcaps=False, fliersize=0, linewidth=1.8, width=0.3, boxprops={"facecolor": "none"}
)

# Manually create legend handles
legend_patches = [mpatches.Patch(color=colors[i], label=methods[i]) for i in range(len(methods))]

# Add legend to the plot
plt.legend(handles=legend_patches, title="Methods", loc="upper right", frameon=True, fontsize=16)
# Increase tick font size
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)

# Add vertical dashed line at 12 seconds
# plt.axvline(x=12, linestyle="dashed", color="black", linewidth=1)

# Manually set y-axis labels to preserve stacking and gaps
#ax.set_yticks(y_ticks)
#ax.set_yticklabels(y_labels)

# Adjust legend position
# plt.legend(title="", frameon=True, loc="upper right")

# Labels and layout
plt.xlabel("# Pedestrians", fontsize=16)
plt.ylabel("Experiment", fontsize=16)
plt.title("Box Plot of Task Duration Across Experiment Sets")
plt.savefig("boxplot_spacing.svg", format="svg", dpi=300)

plt.show()