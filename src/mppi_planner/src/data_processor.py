import os
import json

import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime

class DataProcessor:

    def __init__(self, group_index=-1):

        self.current_dir = os.path.dirname(__file__)
        self.data_dir = os.path.join(self.current_dir, 'data')

        self.all_files = self.load_all_files()
        self.groups = self.group_files_on_time()

        self.to_be_analysed_group = self.groups[group_index]
        self.analyse_group(self.to_be_analysed_group)

    def load_all_files(self):
        files = os.listdir(self.data_dir)
        files.sort()
        return files
    
    def group_files_on_time(self):
        
        groups = []

        last_time = datetime(year=100, month=1, day=1, hour=0, minute=0, second=0)
        
        for file in self.all_files:

            time = file.split('_')[-1]  # Extract time from file name
            time = datetime.strptime(time, '%H%M%S')  # Convert time to datetime object
            time_difference = (time - last_time).total_seconds() / 60  # Calculate the difference in minutes between the last time and time

            if time_difference > 4:  # If the difference between the current time and the last time is greater than 4 minutes, create a new group
                groups.append([file])
            else:  # Otherwise, add the file to the last group
                groups[-1].append(file)
            
            last_time = time
        
        return groups

    def analyse_group(self, group):
        
        processed_data = []

        for file in group:
            
            # Json load the file
            with open(os.path.join(self.data_dir, file), 'r') as f:
                data = json.load(f)
            
            # Extract the data
            processed_data.append(self.analyse_data(data))
                
        planning_times = []
        max_planning_times = []
        durations = []
        mean_velocities = []
        min_distances = []
        collisions = []

        for data in processed_data:
            planning_times.append(data['mean_planning_times'])
            max_planning_times.append(data['max_planning_time'])
            durations.append(data['duration'])
            mean_velocities.append(data['mean_velocity'])
            min_distances.append(data['min_distance'])
            collisions.append(bool(data['collisions']))

        processed_data_dict = {
            'mean_planning_times': np.mean(planning_times),
            'std_planning_times': np.std(planning_times),
            'max_planning_time': np.max(max_planning_times),
            'mean_duration': np.mean(durations),
            'std_duration': np.std(durations),
            'mean_velocity': np.mean(mean_velocities),
            'std_velocity': np.std(mean_velocities),
            'mean_min_distance': np.mean(min_distances),
            'std_min_distance': np.std(min_distances),
            'collisions': np.sum(collisions),
        }

        print(processed_data_dict)
            
            
    
    def analyse_data(self, data):

        processed_data = {
            'mean_planning_times': -1,
            'max_planning_time': -1,
            'duration': -1,
            'mean_velocity': -1,
            'min_distance': 1e6,
            'collisions': 0,
        }

        # Calculate mean and max planning times
        planning_times = np.array(data['planning_times'][1:])
        processed_data['mean_planning_times'] = np.mean(planning_times)
        processed_data['max_planning_time'] = np.max(planning_times)

        # Get the ego robot states
        ego_robot_states = np.array(data['ego_robot_states'])
        x = ego_robot_states[:, 0]
        y = ego_robot_states[:, 1]
        vx = ego_robot_states[:, 3]
        vy = ego_robot_states[:, 4]

        # Pedestrian states
        pedestrian_states = np.array(data['pedestrian_states'])
        x_pedestrian = pedestrian_states[:, :, 0]
        y_pedestrian = pedestrian_states[:, :, 1]

        # Calculate the distance between the ego robot and the pedestrian
        distances = np.sqrt((x[:, np.newaxis] - x_pedestrian)**2 + (y[:, np.newaxis] - y_pedestrian)**2)
        # Get the minimum distance
        processed_data['min_distance'] = np.min(distances) - 0.625

        # Calculate the mean velocity
        v = np.sqrt(vx**2 + vy**2)
        processed_data['mean_velocity'] = np.mean(v)

        # Calculate total travelled distance using x and y
        travelled_distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        total_travelled_distance = np.sum(travelled_distances)

        # Calculate the duration
        duration = total_travelled_distance / processed_data['mean_velocity']
        processed_data['duration'] = duration

        processed_data['duration'] = data['run_time']

        # Get the collisions
        processed_data['collisions'] = data['collisions']

        return processed_data

if __name__ == '__main__':

    select_group = -1

    dp = DataProcessor(group_index=select_group)
