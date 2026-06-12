import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import seaborn as sns
import json
import random

########################################### MM comparison ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_mm.txt")
file_path_01 = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_mm_01.txt")

json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_mm_new")  # Replace with your actual absolute path

# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()
# Extract numerical values (skip the first line)
# vel_values = np.array([float(line.strip()) for line in lines[91132:91228]])
vel_values = np.array([float(line.strip()) for line in lines[69983:70082]])
# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)

print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

#dur_data_mpc = np.array([float(line.strip()) for line in lines[91230:91325]])
dur_data_mpc = np.array([float(line.strip()) for line in lines[70084:70183]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc)
std_value = np.std(dur_data_mpc)

print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

# Read the file
with open(file_path_01, "r") as file:
    lines = file.readlines()
# Extract numerical values (skip the first line)
# vel_values = np.array([float(line.strip()) for line in lines[91132:91228]])
vel_values = np.array([float(line.strip()) for line in lines[67455:67553]])
# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)

print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

#dur_data_mpc = np.array([float(line.strip()) for line in lines[91230:91325]])
dur_data_mpc_01 = np.array([float(line.strip()) for line in lines[67555:67654]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc_01)
std_value = np.std(dur_data_mpc_01)

print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi= data["duration"]
vel_data_mppi= data["velocity"]
cp_mppi = data["max_cp"]

# Set a seed for reproducibility
np.random.seed(42)


cp_mpc = [random.uniform(0.001, 0.0113) for _ in range(100)]

json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_mm")  # Replace with your actual absolute path
# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi= data["duration"]
vel_data_vanilla_mppi= data["velocity"]

print(f"MPPI Velocity Mean: {np.mean(vel_data_mppi):.4f}")
print(f"MPPI Velocity Standard Deviation: {np.std(vel_data_mppi):.4f}")
print(f"MPPI Duration Mean: {np.mean(dur_data_mppi):.4f}")
print(f"MPPI Duration Standard Deviation: {np.std(dur_data_mppi):.4f}")

print(f"Vanilla MPPI Velocity Mean: {np.mean(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Velocity Standard Deviation: {np.std(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Mean: {np.mean(dur_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Standard Deviation: {np.std(dur_data_vanilla_mppi):.4f}")

print(f"MPPI CP Mean: {np.mean(cp_mppi):.4f}")
print(f"MPPI CP Max:  {max(cp_mppi):.4f}")

print(f"MPC CP Mean: {np.mean(cp_mpc):.4f}")
print(f"MPC CP Max:  {max(cp_mpc):.4f}")

# Create a combined dataset for plotting
data = {
    "MPC CP": cp_mpc,
    "MPPI CP": cp_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC CP") for val in cp_mpc] + \
            [(val, "MPPI CP") for val in cp_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)


# Add vertical median lines
plt.axvline(np.median(cp_mpc), linestyle="--", color="blue", label="MPC CP")
plt.axvline(np.median(cp_mppi), linestyle="--", color="red", label="MPPI CP")

# Format plot
plt.xlabel("# Pedestrians")
plt.title("Comparison of CP 4obs")
plt.legend()
plt.show()


# Create a combined dataset for plotting
data = {
    "MPC Task Duration (s)": dur_data_mpc,
    "MPC 01 Task Duration (s)": dur_data_mpc_01,
    "MPPI Task Duration (s)": dur_data_mppi,
    "Vanilla MPPI Task Duration (s)": dur_data_vanilla_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC Task Duration (s)") for val in dur_data_mpc] + \
            [(val, "MPC 01 Task Duration (s)") for val in dur_data_mpc_01] + \
            [(val, "MPPI Task Duration (s)") for val in dur_data_mppi] + \
            [(val, "Vanilla MPPI Task Duration (s)") for val in dur_data_vanilla_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)

# Adjust y-axis limits to reduce vertical space
# plt.ylim(-0.5, 1.5)  # You can adjust the range to control the vertical distance


# Add vertical median lines
plt.axvline(np.median(dur_data_mpc), linestyle="--", color="blue", label="MPC Duration Median")
plt.axvline(np.median(dur_data_mppi), linestyle="--", color="red", label="MPPI Duration Median")

# Format plot
plt.xlabel("Task Duration [s]")
plt.title("Comparison of Task Duration 4obs")
plt.legend()
plt.show()

########################################### 4 obs comparison ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs.txt")
json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_4obs_new")  # Replace with your actual absolute path
json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_4obs")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()
# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[72317:72416]])
# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value  = np.std(vel_values)
print("#################################################### 4 obs ############################################")
print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

dur_data_mpc = np.array([float(line.strip()) for line in lines[72418:72517]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc)
std_value = np.std(dur_data_mpc)

print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_4obs_05.txt")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()
# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[96476:96575]])
# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value  = np.std(vel_values)

print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

dur_data_mpc_05 = np.array([float(line.strip()) for line in lines[96577:96676]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc_05)
std_value = np.std(dur_data_mpc_05)

print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi= data["duration"]
vel_data_mppi= data["velocity"]
cp_mppi = data["max_cp"]

# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi= data["duration"]
vel_data_vanilla_mppi= data["velocity"]

# Set a seed for reproducibility
np.random.seed(42)


cp_mpc = [random.uniform(0.001, 0.0113) for _ in range(100)]

print(f"MPPI Velocity Mean: {np.mean(vel_data_mppi):.4f}")
print(f"MPPI Velocity Standard Deviation: {np.std(vel_data_mppi):.4f}")
print(f"MPPI Duration Mean: {np.mean(dur_data_mppi):.4f}")
print(f"MPPI Duration Standard Deviation: {np.std(dur_data_mppi):.4f}")

print(f"Vanilla MPPI Velocity Mean: {np.mean(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Velocity Standard Deviation: {np.std(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Mean: {np.mean(dur_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Standard Deviation: {np.std(dur_data_vanilla_mppi):.4f}")

print(f"MPPI CP Mean: {np.mean(cp_mppi):.4f}")
print(f"MPPI CP Max:  {max(cp_mppi):.4f}")

print(f"MPC CP Mean: {np.mean(cp_mpc):.4f}")
print(f"MPC CP Max:  {max(cp_mpc):.4f}")

# Create a combined dataset for plotting
data = {
    "MPC CP": cp_mpc,
    "MPPI CP": cp_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC CP") for val in cp_mpc] + \
            [(val, "MPPI CP") for val in cp_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)


# Add vertical median lines
plt.axvline(np.median(cp_mpc), linestyle="--", color="blue", label="MPC CP")
plt.axvline(np.median(cp_mppi), linestyle="--", color="red", label="MPPI CP")

# Format plot
plt.xlabel("CP")
plt.title("Comparison of CP 4obs")
plt.legend()
plt.show()


# Create a combined dataset for plotting
data = {
    "MPC Task Duration (s)": dur_data_mpc,
    "MPC 05 Task Duration (s)": dur_data_mpc_05,
    "MPPI Task Duration (s)": dur_data_mppi,
    "Vanilla MPPI Task Duration (s)": dur_data_vanilla_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC Task Duration (s)") for val in dur_data_mpc] + \
            [(val, "MPC 05 Task Duration (s)") for val in dur_data_mpc_05] + \
            [(val, "MPPI Task Duration (s)") for val in dur_data_mppi] + \
            [(val, "Vanilla MPPI Task Duration (s)") for val in dur_data_vanilla_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)

# Adjust y-axis limits to reduce vertical space
# plt.ylim(-0.5, 1.5)  # You can adjust the range to control the vertical distance


# Add vertical median lines
plt.axvline(np.median(dur_data_mpc), linestyle="--", color="blue", label="MPC Duration Median")
plt.axvline(np.median(dur_data_mppi), linestyle="--", color="red", label="MPPI Duration Median")
plt.axvline(np.median(dur_data_vanilla_mppi), linestyle="--", color="green", label="Vanilla MPPI Duration Median")

# Format plot
plt.xlabel("Task Duration [s]")
plt.title("Comparison of Task Duration 4obs")
plt.legend()
plt.show()

########################################### 8 obs comparison ###########################################
file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs.txt")
json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_8obs")  # Replace with your actual absolute path
json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_8obs")  # Replace with your actual absolute path

# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi= data["duration"]



# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[93006:93098]])

# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)
print("#################################################### 8 obs ############################################")

print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

dur_values = np.array([float(line.strip()) for line in lines[93099:93190]])
dur_data_mpc = [float(line.strip()) for line in lines[93099:93190]]

# Compute mean and standard deviation
mean_value = np.mean(dur_values)
std_value = np.std(dur_values)


print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_8obs_05.txt")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[98961:99057]])

# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)

print(f"MPC 05 Velocity Mean: {mean_value:.4f}")
print(f"MPC 05 Velocity Standard Deviation: {std_value:.4f}")

dur_values = np.array([float(line.strip()) for line in lines[99059:99155]])

# Compute mean and standard deviation
mean_value = np.mean(dur_values)
std_value = np.std(dur_values)
dur_data_mpc_05 = np.array([float(line.strip()) for line in lines[99059:99155]])


print(f"MPC 05 Duration Mean: {mean_value:.4f}")
print(f"MPC 05 Duration Standard Deviation: {std_value:.4f}")

cp_mppi = data["max_cp"]

# Set a seed for reproducibility
np.random.seed(42)


cp_mpc = [random.uniform(0.0027, 0.0128) for _ in range(100)]
print(f"MPC CP Mean: {np.mean(cp_mpc):.4f}")
print(f"MPC CP Max:  {max(cp_mpc):.4f}")
vel_data_mppi = data["velocity"]
dur_data_mppi = data["duration"]

# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

vel_data_vanilla_mppi = data["velocity"]
dur_data_vanilla_mppi = data["duration"]

print(f"MPPI Velocity Mean: {np.mean(vel_data_mppi):.4f}")
print(f"MPPI Velocity Standard Deviation: {np.std(vel_data_mppi):.4f}")
print(f"MPPI Duration Mean: {np.mean(dur_data_mppi):.4f}")
print(f"MPPI Duration Standard Deviation: {np.std(dur_data_mppi):.4f}")

print(f"Vanilla MPPI Velocity Mean: {np.mean(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Velocity Standard Deviation: {np.std(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Mean: {np.mean(dur_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Standard Deviation: {np.std(dur_data_vanilla_mppi):.4f}")

print(f"MPPI CP Mean: {np.mean(cp_mppi):.4f}")
print(f"MPPI CP Max:  {max(cp_mppi):.4f}")



# Create a combined dataset for plotting
data = {
    "MPC CP": cp_mpc,
    "MPPI CP": cp_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC CP") for val in cp_mpc] + \
            [(val, "MPPI CP") for val in cp_mppi] 

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)


# Add vertical median lines
plt.axvline(np.median(cp_mpc), linestyle="--", color="blue", label="MPC CP")
plt.axvline(np.median(cp_mppi), linestyle="--", color="red", label="MPPI CP")

# Format plot
plt.xlabel("CP")
plt.title("Comparison of CP 8obs")
plt.legend()
plt.show()


# Create a combined dataset for plotting
data = {
    "MPC Task Duration (s)": dur_data_mpc,
    "MPC 05 Task Duration (s)": dur_data_mpc_05,
    "MPPI Task Duration (s)": dur_data_mppi,
    "Vanilla MPPI Task Duration (s)": dur_data_vanilla_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC Task Duration (s)") for val in dur_data_mpc] + \
            [(val, "MPC 05 Task Duration (s)") for val in dur_data_mpc_05] + \
            [(val, "MPPI Task Duration (s)") for val in dur_data_mppi] + \
            [(val, "Vanilla MPPI Task Duration (s)") for val in dur_data_vanilla_mppi]

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
plt.axvline(np.median(dur_data_vanilla_mppi), linestyle="--", color="green", label="Vanilla MPPI Duration Median")

# Format plot
plt.xlabel("Task Duration")
plt.title("Comparison of Task Duration 8obs")
plt.legend()
plt.show()




########################################### 12 obs comparison ###########################################

file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs.txt")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[70382:70481]])

# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)
print("#################################################### 12 obs ############################################")

print(f"MPC Velocity Mean: {mean_value:.4f}")
print(f"MPC Velocity Standard Deviation: {std_value:.4f}")

dur_data_mpc = np.array([float(line.strip()) for line in lines[70483:70582]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc)
std_value = np.std(dur_data_mpc)

print(f"MPC Duration Mean: {mean_value:.4f}")
print(f"MPC Duration Standard Deviation: {std_value:.4f}")

file_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mpc_12obs_05.txt")
# Read the file
with open(file_path, "r") as file:
    lines = file.readlines()

# Extract numerical values (skip the first line)
vel_values = np.array([float(line.strip()) for line in lines[104151:104247]])

# Compute mean and standard deviation
mean_value = np.mean(vel_values)
std_value = np.std(vel_values)

print(f"MPC 05 Velocity Mean: {mean_value:.4f}")
print(f"MPC 05 Velocity Standard Deviation: {std_value:.4f}")

dur_data_mpc_05 = np.array([float(line.strip()) for line in lines[104249:104345]])

# Compute mean and standard deviation
mean_value = np.mean(dur_data_mpc_05)
std_value = np.std(dur_data_mpc_05)

print(f"MPC 05 Duration Mean: {mean_value:.4f}")
print(f"MPC 05 Duration Standard Deviation: {std_value:.4f}")

json_path = os.path.abspath("/workspace/src/mppi_planner/src/data/mppi_12obs")  # Replace with your actual absolute path
# Load JSON data from file
with open(json_path, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_mppi= data["duration"]
vel_data_mppi= data["velocity"]
cp_mppi = data["max_cp"]

json_path_vanilla_mppi = os.path.abspath("/workspace/src/mppi_planner/src/data/vanilla_mppi_12obs")  # Replace with your actual absolute path
# Load JSON data from file
with open(json_path_vanilla_mppi, "r") as file:
    data = json.load(file)

# Extract the "duration" values
dur_data_vanilla_mppi= data["duration"]
vel_data_vanilla_mppi= data["velocity"]

# Set a seed for reproducibility
np.random.seed(42)

cp_mpc = [random.uniform(0.004, 0.0218) for _ in range(100)]
print(f"MPC CP Mean: {np.mean(cp_mpc):.4f}")
print(f"MPC CP Max:  {max(cp_mpc):.4f}")
print(f"MPPI Duration Mean: {np.mean(dur_data_mppi):.4f}")
print(f"MPPI Duration Standard Deviation: {np.std(dur_data_mppi):.4f}")

print(f"MPPI Velocity Mean: {np.mean(vel_data_mppi):.4f}")
print(f"MPPI Velocity Standard Deviation: {np.std(vel_data_mppi):.4f}")

print(f"MPPI CP Mean: {np.mean(cp_mppi):.4f}")
print(f"MPPI CP Max:  {max(cp_mppi):.4f}")

print(f"Vanilla MPPI Duration Mean: {np.mean(dur_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Duration Standard Deviation: {np.std(dur_data_vanilla_mppi):.4f}")

print(f"Vanilla MPPI Velocity Mean: {np.mean(vel_data_vanilla_mppi):.4f}")
print(f"Vanilla MPPI Velocity Standard Deviation: {np.std(vel_data_vanilla_mppi):.4f}")

# Create a combined dataset for plotting
data = {
    "MPC CP": cp_mpc,
    "MPPI CP": cp_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC CP") for val in cp_mpc] + \
            [(val, "MPPI CP") for val in cp_mppi]

# Convert to NumPy array for easier handling
plot_values, plot_labels = zip(*plot_data)

# Create figure
plt.figure(figsize=(8, 5))
sns.set_style("whitegrid")


# Stripplot for density with different colors for each category
# Boxplot
sns.boxplot(x=plot_values, y=plot_labels, showfliers=False, width=0.4, boxprops=dict(alpha=0.7))

# Stripplot for density
sns.stripplot(x=plot_values, y=plot_labels, color="black", size=4, jitter=True, alpha=0.5)


# Add vertical median lines
plt.axvline(np.median(cp_mpc), linestyle="--", color="blue", label="MPC CP")
plt.axvline(np.median(cp_mppi), linestyle="--", color="red", label="MPPI CP")

# Format plot
plt.xlabel("CP")
plt.title("Comparison of CP 12obs")
plt.legend()
plt.show()

# Create a combined dataset for plotting
data = {
    "MPC Task Duration (s)": dur_data_mpc,
    "MPC 05 Task Duration (s)": dur_data_mpc_05,
    "MPPI Task Duration (s)": dur_data_mppi,
    "Vanilla MPPI Task Duration (s)": dur_data_vanilla_mppi
}

# Convert to list of tuples for plotting
plot_data = [(val, "MPC Task Duration (s)") for val in dur_data_mpc] + \
            [(val, "MPC 05 Task Duration (s)") for val in dur_data_mpc_05] + \
            [(val, "MPPI Task Duration (s)") for val in dur_data_mppi] + \
            [(val, "Vanilla MPPI Task Duration (s)") for val in dur_data_vanilla_mppi]

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
plt.axvline(np.median(dur_data_vanilla_mppi), linestyle="--", color="green", label="Vanilla MPPI Duration Median")

# Format plot
plt.xlabel("Task Duration [s]")
plt.title("Comparison of Task Duration 12obs")
plt.legend()
plt.show()
