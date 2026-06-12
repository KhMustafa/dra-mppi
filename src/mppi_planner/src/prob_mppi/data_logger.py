import os
import json
import numpy as np
from datetime import datetime

class DataLogger:
    def __init__(self, num_experiments):
        self.num_experiments = num_experiments
        self.experiment_counter = 0
        self.data_dict = {
            "collisions": [],
            "max_cp": [],
            "max_joint_mc": [],
            "min_min_dist": [],
            "planning_time": [],
            "travelled_distance": [],
            "velocity": [],
            "duration": []
        }
        self.reset_data()

    def reset_data(self):
        self.computational_time = []
        self.step_num = 0
        self.max_cp = 1e-6
        self.max_joint_mc = 1e-6
        self.dist_min = 1e6

        self.collision_detected = False

    def log_computation_time(self, start_time, end_time):
        if self.step_num > 0:
            self.computational_time.append(end_time - start_time)
        self.step_num += 1

    def log_step(self, robot_state, state_coordinates, cp, cp_joint):
        min_dist = 1e06

        for state_coordinate in state_coordinates:

            x_obst = state_coordinate[0]
            y_obst = state_coordinate[1]
            x_ego = robot_state[0]
            y_ego = robot_state[1]
            dist = ((x_ego - x_obst)**2 + (y_ego - y_obst)**2)**0.5
            if dist < min_dist:
                min_dist = dist
            
            coll_dist = 0.625
            if dist < coll_dist and not self.collision_detected:
                self.data_dict["collisions"].append(True)
                self.collision_detected = True
        
        # self.data_dict["min_dist"].append(min_dist.item())
        # self.data_dict["cp"].append(cp)
        # print(type(min_dist))
        # self.data_dict["joint_mc"].append(cp_joint)

        self.max_cp = max(self.max_cp, cp)
        self.max_joint_mc = max(self.max_joint_mc, cp_joint)
        self.dist_min = min(self.dist_min, min_dist)

    
    def finalize_experiment(self, robot_vel_vec, travelled_distance):
        if len(self.computational_time) > 10:
            self.data_dict["planning_time"].append(np.mean(self.computational_time) * 1000)
            self.data_dict["travelled_distance"].append(travelled_distance)
            self.data_dict["max_cp"].append(self.max_cp)
            self.data_dict["max_joint_mc"].append(self.max_joint_mc)
            self.data_dict["min_min_dist"].append(self.dist_min)
            self.data_dict["velocity"].append(np.mean(robot_vel_vec))
            self.data_dict["duration"].append(travelled_distance / np.mean(robot_vel_vec))

            if not self.collision_detected:
                self.data_dict["collisions"].append(False)
            
            self.reset_data()

    def aggregate_results(self):
        x = 4
        self.data_dict["avg_velocity"] = np.mean(self.data_dict["velocity"])
        self.data_dict["std_velocity"] = np.std(self.data_dict["velocity"])
        self.data_dict["avg_duration"] = np.mean(self.data_dict["duration"])
        self.data_dict["std_duration"] = np.std(self.data_dict["duration"])
        self.data_dict["avg_planning_time"] = np.mean(self.data_dict["planning_time"])
        self.data_dict["std_planning_time"] = np.std(self.data_dict["planning_time"])
        self.data_dict["number_of_collisions"] = sum(self.data_dict["collisions"])

    def save_data(self, current_dir):
        formatted_time = datetime.now().strftime("%m%d_%H%M%S")
        data_path = os.path.join(current_dir, "data", f"Jackal_{formatted_time}")
        with open(data_path, 'w') as json_file:
            json.dump(self.data_dict, json_file, indent=4)
        print(f"Data saved at {data_path}")
