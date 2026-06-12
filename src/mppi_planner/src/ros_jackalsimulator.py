#!/usr/bin/python3

import rospy
import gc
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
import numpy as np
import time
import sys
from planner.mppi import MPPIPlanner

import yaml
from yaml import SafeLoader
import torch


from interfaces.ros_jackalsimulator_interface import JackalInterface
from prob_mppi.data_logger import DataLogger


from datetime import datetime
import math
import json

import os
current_dir = os.path.dirname(os.path.realpath(__file__))
prob_mppi_dir = os.path.join(current_dir, 'prob_mppi')
sys.path.append(prob_mppi_dir)
sys.path.append(os.path.dirname(os.path.dirname(current_dir)))

sys.path.append(current_dir)

from prob_mppi.dynamic_obstacles import DynamicObstacles
from prob_mppi.jackal_objective import JackalObjective
from prob_mppi.jackal_dynamics import JackalDynamics
from prob_mppi.priors import Priors

from utils.spline import TorchSpline2D 



# class ReferencePath:
#     def __init__(self, msg):
#         # Initialize the attributes with empty lists
#         self.x = []
#         self.y = []
#         self.psi = []
#         self.v = []
#         self.s = []

#         for pose in msg.poses:
#             self.x.append(pose.pose.position.x)
#             self.y.append(pose.pose.position.y)
        
        #print("reference path x is:", self.x)
        #print("reference path y is:", self.y)


class Planner:
    def __init__(self):
        self.cfg = self.new_load_config()
        self.interface = JackalInterface(self.cfg)
        self.travelled_distance = 0
        self.prev_x = 0
        self.prev_y = 0
        self.experiment_counter = 0 
        self.robot_s = 0
        self.last_call_time  = None
        self.num_experiments = 4
        self.data_logger = DataLogger(num_experiments=self.num_experiments)     # initialize DataLogger instance
        self.robot_vel_vec = []

        
    def new_load_config(self):
        yaml_path = os.path.join(prob_mppi_dir, 'run_sim_jackal.yaml')
        with open(yaml_path, "r") as file:
            config = yaml.safe_load(file)
        return config

    
    def reformat_odom(self, robot_pos, robot_vel):
        x = robot_pos[0]
        y = robot_pos[1]
        theta = robot_pos[2]
        v = robot_vel[0]
        omega = robot_vel[1]
        # Record the current time
        current_time = time.perf_counter()
    
        if self.last_call_time is not None:
            # Calculate time elapsed since the last call
            elapsed_time = current_time - self.last_call_time
            self.robot_s += v * elapsed_time

        self.last_call_time = current_time


        state = torch.tensor([x, y, theta, v, self.robot_s, omega], device=self.cfg["device"], dtype=torch.float32)
        return state

    def init_predicted_tensors(self, obst, exp_mode):

        if exp_mode == 'simulation':
            self.state_coordinates = torch.zeros(len(obst.obstacles), 2, device=self.cfg["device"])
            self.predicted_cov = torch.zeros(self.cfg["mppi"]["horizon"], 2, 2, device=self.cfg["device"])
            

            self.prediction_dict = {}
            for obst_index, obstacles in enumerate(obst.obstacles):
                predicted_coordinates = torch.zeros(self.cfg["mppi"]["horizon"], len(obstacles.gaussians), 2,
                                                    device=self.cfg["device"])
                predicted_covs = torch.zeros(self.cfg["mppi"]["horizon"], len(obstacles.gaussians), 2, 2,
                                             device=self.cfg["device"])
                self.prediction_dict[obst_index] = {
                "predicted_coordinates": predicted_coordinates,
                "predicted_covs": predicted_covs,
                }


    def reformat_obstacles_odom(self, obst, exp_mode):
        #print("number of modes: ", len(obst.obstacles[0].gaussians))
        if obst is not None:
            if exp_mode == 'simulation':
                for t in range(self.cfg["mppi"]["horizon"]):
                    self.predicted_cov[t, 0, 0] = 0.3
                    self.predicted_cov[t, 1, 1] = 0.3

                # iterate over the obstacles in the workspace
                for obst_index, obstacles in enumerate(obst.obstacles):
                    # Iterate over the time steps
                    for t in range(self.cfg["mppi"]["horizon"] + 1):

                        #print("number of predicted modes: ", len(obstacles.gaussians))
                        # iterate over the predicred modes
                        for mode in range(len(obstacles.gaussians)):
                            #print("number of modes is: ", len(obstacles.gaussians))
                            if t == 0:
                                self.state_coordinates[obst_index, 0] = obstacles.pose.position.x
                                self.state_coordinates[obst_index, 1] = obstacles.pose.position.y
                            else:
                                self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, mode, 0] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.x
                                self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, mode, 1] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.y
                                self.prediction_dict[obst_index]["predicted_covs"][t - 1, mode,:] = self.predicted_cov[t - 1, :, :]
                                    
    
    def rotation_matrix_from_heading(self, heading: float) -> np.ndarray:
        return np.array([
            [np.cos(heading),  np.sin(heading)],
            [-np.sin(heading), np.cos(heading)]
        ])

    def quaternion_to_angle(self, q: Quaternion) -> float:
        """Convert a quaternion to a yaw (rotation around the Z-axis)."""
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        return np.arctan2(t3, t4)
    
    def extract_robot_state(self):
        orientation_quaternion = self.interface.odom_msg.pose.pose.orientation
        orientation_euler = euler_from_quaternion([
            orientation_quaternion.x,
            orientation_quaternion.y,
            orientation_quaternion.z,
            orientation_quaternion.w
        ])
        yaw = orientation_euler[2]
        robot_position = [
            self.interface.odom_msg.pose.pose.position.x,
            self.interface.odom_msg.pose.pose.position.y,
            yaw, 0
        ]
        robot_velocity = [
            self.interface.odom_msg.twist.twist.linear.x,
            self.interface.odom_msg.twist.twist.linear.y,
            self.interface.odom_msg.twist.twist.angular.z
        ]
        robot_vel = math.sqrt(robot_velocity[0]**2 + robot_velocity[1]**2)
        return robot_position, robot_velocity, robot_vel   

    def update_travelled_distance(self, robot_position):
        if robot_position[0] > 30:
            self.travelled_distance += ((robot_position[0] - self.prev_x)**2 + (robot_position[1] - self.prev_y)**2)**0.5
            self.prev_x = robot_position[0]
            self.prev_y = robot_position[1]  

    def handle_experiment_completion(self):
        if len(self.data_logger.computational_time) > 10:
            self.data_logger.finalize_experiment(self.robot_vel_vec, self.travelled_distance)
            
            self.travelled_distance = 0
            self.prev_x = 0
            self.prev_y = 0
            self.interface.reset_env()
            print("experiment counter", self.experiment_counter)
            
            if self.experiment_counter == self.num_experiments:
                self.data_logger.aggregate_results()
                self.data_logger.save_data(current_dir)
                print("Experiments Completed")
                assert self.experiment_counter < self.num_experiments, "Stopping the planner."
            self.experiment_counter += 1   
    
    def run_jackal_robot(self):
        # Create a publisher for velocity commands
        while self.interface.obstacle_predictions is None or self.interface.ref_path_msg is None:
            # The code will keep looping until we get obstacle predictions and path information
            # print("None")
            pass

        ## determine either we are in simulation mode or real-world experiment mode
        exp_mode = 'simulation'      # it is either simulation or realworld
        
        self.init_predicted_tensors(self.interface.obstacle_predictions, exp_mode)


        step_num = 0

        first_timestep = True


        while not rospy.is_shutdown():
            if self.interface.spline is not None:

                if first_timestep:
                    # gc.collect()
                    self.reformat_obstacles_odom(self.interface.obstacle_predictions, exp_mode)
                    #print("reformating completed")
                    x = self.state_coordinates[:, 0]
                    y = self.state_coordinates[:, 1]
                    headings = torch.zeros_like(x)
                    cov = torch.eye(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"],
                                    dtype=torch.float32).repeat(self.cfg["obstacles"]["num_obstacles"], 1, 1) * \
                        self.cfg["obstacles"]["initial_covariance"]
                    vx = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
                    vy = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
                    
                    obstacles_new = DynamicObstacles(self.cfg, x, y, headings, cov, vx, vy)

                    objective = JackalObjective(self.cfg, plot=False, dyn_obstacles=obstacles_new, 
                                                spline=self.interface.spline)
                    objective.update_goal([60, 0.0])

                    dynamics  = JackalDynamics(self.cfg)
                    priors = Priors(self.cfg)
                    planner = MPPIPlanner(
                        cfg=self.cfg["mppi"],
                        nx=6,
                        dynamics=dynamics.step,
                        running_cost=objective.compute,
                        prior=priors.compute_priors,
                    )

                    first_timestep = False

                    ros_mode_probabilities = torch.tensor([1], device=self.cfg["device"])
                    obstacles_new.set_mode_probabilities(self.predicted_cov, ros_mode_probabilities)


                if self.interface.odom_msg is not None:
                    robot_position, robot_velocity, robot_vel = self.extract_robot_state()
                    self.robot_vel_vec.append(robot_vel)


                    self.reformat_obstacles_odom(self.interface.obstacle_predictions, exp_mode)
                    obstacles_new.state_coordinates = self.state_coordinates
                    obstacles_new.update_multi_mode_predicted_states(self.prediction_dict)


                    robot_state = self.reformat_odom(robot_position, robot_velocity)
                    start_time = time.perf_counter()
                    
                    action_seq, all_samples, all_samples_cost = planner.command(robot_state)
                    
                    end_time = time.perf_counter()
                    print("computation time is: ", end_time - start_time)
                    if step_num > 0:
                        self.data_logger.log_computation_time(start_time, end_time)
                    step_num += 1


                    action = action_seq[0]

                    # publish planned trajectory
                    plan = dynamics.get_trajectory(robot_state, action_seq)
                    self.interface.publish_planned_trajectory(plan)

                    # actuate robot
                    self.interface.actuate(action)
                    time_step = 0

                
                    if robot_position[0] > 40:
                        # this marks the end of one simulation
                        self.handle_experiment_completion()


if __name__ == '__main__':
    try:
        planner = Planner()
        planner.run_jackal_robot()
    except rospy.ROSInterruptException:
        pass
