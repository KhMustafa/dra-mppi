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
from torch.autograd import Variable
from scipy.optimize import minimize
from interfaces.ros_dinova_interface import DinovaInterface

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
from prob_mppi.dinova_objective  import DinovaObjective
from prob_mppi.dinova_dynamics   import DinovaDynamics
from prob_mppi.priors import Priors

from utils.spline import TorchSpline2D


class Planner:
    def __init__(self):
        # config = self.load_config()
        # self.cfg = OmegaConf.to_object(config)
        self.cfg = self.new_load_config()
        self.interface = DinovaInterface(self.cfg)
        self.travelled_distance = 0
        self.prev_x = 0
        self.prev_y = 0
        self.experiment_counter = 0 
        self.rotating_to_goal = False

        
    def new_load_config(self):
        yaml_path = os.path.join(prob_mppi_dir, 'run_real_dinova.yaml')
        with open(yaml_path, "r") as file:
            config = yaml.safe_load(file)
        return config

    
    def reformat_odom(self, robot_pos, robot_vel):

        x = robot_pos[0]
        y = robot_pos[1]
        theta = robot_pos[2]
        # v = robot_vel[0]
        omega = robot_vel[2]

        vx = robot_vel[0]
        vy = robot_vel[1]

        state = torch.tensor([x, y, theta, vx, vy, omega], device=self.cfg["device"], dtype=torch.float32)
        return state

    def init_predicted_tensors(self, obst, exp_mode):

        if exp_mode == 'simulation':
            self.state_coordinates = torch.zeros(len(obst.obstacles), 2, device=self.cfg["device"])
            self.predicted_cov = torch.zeros(self.cfg["mppi"]["horizon"], 2, 2, device=self.cfg["device"])


            self.prediction_dict = {}
            for obst_index, obstacles in enumerate(obst.obstacles):
                predicted_coordinates = torch.zeros(self.cfg["mppi"]["horizon"], len(obstacles.gaussians), 2,
                                                    device=self.cfg["device"])
                self.prediction_dict[obst_index] = {
                "predicted_coordinates": predicted_coordinates,
                "crossed": False,
                }
        elif exp_mode == 'realworld':
            self.state_coordinates = torch.zeros(len(obst.objects), 2, device=self.cfg["device"])
            self.predicted_cov = torch.zeros(self.cfg["mppi"]["horizon"], 2, 2, device=self.cfg["device"])
            
            self.prediction_dict = {}
            objects = [obj for obj in obst.objects if obj.id != 0]

            for obst_index, obstacle in enumerate(objects):
                # replace len(obstacles.gaussians) to 1 since we have a single mode in realworld experiments
                if (obstacle.id != 0):
                    predicted_coordinates = torch.zeros(self.cfg["mppi"]["horizon"], 1, 2,
                                                    device=self.cfg["device"])
                    predicted_covs = torch.zeros(self.cfg["mppi"]["horizon"], 1, 2, 2,
                                                device=self.cfg["device"])
                    self.prediction_dict[obst_index] = {
                    "predicted_coordinates": predicted_coordinates,
                    "predicted_covs": predicted_covs,
                    "crossed": False,
                    }


    def reformat_obstacles_odom(self, obst, exp_mode):
        if obst is not None:
            if exp_mode == 'simulation':
                for t in range(self.cfg["mppi"]["horizon"]):
                    # self.predicted_cov[t, :, :] = [[, 0],
                    #      [0, ]]
                    #self.predicted_cov[t, 0, 0] = obst.obstacles[0].gaussians[0].major_semiaxis[0]
                    #self.predicted_cov[t, 1, 1] = obst.obstacles[0].gaussians[0].major_semiaxis[0]
                    self.predicted_cov[t, 0, 0] = 0.3
                    self.predicted_cov[t, 1, 1] = 0.3

                # iterate over the obstacles in the workspace
                for obst_index, obstacles in enumerate(obst.obstacles):

                    # Iterate over the time steps
                    for t in range(self.cfg["mppi"]["horizon"] + 1):

                        # print("number of predicted modes: ", len(obstacles.gaussians))

                        # iterate over the predicred modes
                        for mode in range(len(obstacles.gaussians)):

                            if t == 0:
                                self.state_coordinates[obst_index, 0] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.x
                                self.state_coordinates[obst_index, 1] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.y
                            else:
                                self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, mode, 0] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.x
                                self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, mode, 1] = obstacles.gaussians[mode].mean.poses[
                                    t].pose.position.y
                            '''
                            if obstacle_index == 0:
                                predicted_cov[t - 1, :, :] = torch.tensor(
                                    [[obstacles.gaussians[mode].major_semiaxis[0], 0],
                                     [0, obstacles.gaussians[mode].major_semiaxis[0]]], device=self.cfg["device"])
                            '''
            elif exp_mode == 'realworld':
                for t in range(self.cfg["mppi"]["horizon"]):
                    # self.predicted_cov[t, :, :] = [[, 0],
                    #      [0, ]]
                    #self.predicted_cov[t, 0, 0] = obst.obstacles[0].gaussians[0].major_semiaxis[0]
                    #self.predicted_cov[t, 1, 1] = obst.obstacles[0].gaussians[0].major_semiaxis[0]
                    self.predicted_cov[t, 0, 0] = 0.1
                    self.predicted_cov[t, 1, 1] = 0.1

                positions  = []
                radii      = []
                twists     = []

                objects = [obj for obj in obst.objects if obj.id != 0]


                # iterate over the obstacles in the workspace
                for obst_index, object in enumerate(objects):


                    if object.id != 0:
                        prev_size = len(positions)
                        positions.append(np.array([object.pose.position.x, object.pose.position.y]))
                        radii.append(object.shape.dimensions[1])

                        new_obstacles = len(positions) - prev_size
                        global_twist      = object.twist
                        '''
                        if object.id == 4:
                            print("ekf returned twist x: ", global_twist.linear.x)
                            print("ekf returned twist y: ", global_twist.linear.y)
                        '''

                        rot_matrix        = self.rotation_matrix_from_heading(-self.quaternion_to_angle(object.pose.orientation))
                        twist_vector      = np.array([global_twist.linear.x, global_twist.linear.y])
                        twist_out         = rot_matrix @ twist_vector  # Matrix-vector multiplication

                        for _ in range(new_obstacles):
                            twists.append(twist_out)

                idx = 0
                for position, twist in zip(positions, twists):
                        # Iterate over the time steps
                        #print("obstacle velocity vx is: ", twist[0])
                        #print("obstacle velocity vy is: ", twist[1])
                        #print("obstacle position x is: ", position[0])
                        #print("obstacle position y is: ", position[1])

                        for t in range(self.cfg["mppi"]["horizon"] + 1):
                                if t == 0:
                                    self.state_coordinates[idx, 0] = position[0]
                                    self.state_coordinates[idx, 1] = position[1]
                                else:
                                    self.prediction_dict[idx]["predicted_coordinates"][t - 1, 0, 0] = position[0] + twist[0] * self.cfg["mppi"]["dt"] * t
                                    self.prediction_dict[idx]["predicted_coordinates"][t - 1, 0, 1] = position[1] + twist[1] * self.cfg["mppi"]["dt"] * t
                                    self.prediction_dict[idx]["predicted_covs"][t - 1, 0, :] = self.predicted_cov[t-1, :, :]
                                    #print("time step: ", t)
                                    if idx == 1:
                                        '''
                                        print("predicted dict x coordinate: ", self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, 0, 0])
                                        print("predicted dict y coordinate: ", self.prediction_dict[obst_index]["predicted_coordinates"][t - 1, 0, 1])
                                        '''
                        '''
                        if idx == 1:
                            print("twist_x: ", twist[0])
                            print("twist_y: ", twist[1])
                        '''
                        idx += 1        


        return self.state_coordinates, self.predicted_cov, self.prediction_dict
    
    
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
    
    def rotate_to_goal(self, goal, pose, yaw):
        goal_angle = torch.atan2(goal[1] - pose[1], goal[0] - pose[0])
        angle = torch.remainder(goal_angle-yaw, 2*torch.pi)
        if angle > torch.pi:
            angle -= 2*torch.pi
        # print("Angle to goal", angle, goal_angle)
        if torch.abs(angle) < 0.2:
            self.rotating_to_goal = False
        self.interface.actuate(torch.tensor([0, torch.sign(angle)*1.5, 0.0]))
        
          
    def run_dinova_robot(self):
        # Create a publisher for velocity commands
        while self.interface.obstacle_predictions is None or self.interface.ref_path_msg is None:
            # The code will keep looping until we get obstacle predictions and path information
            pass

        ## determine either we are in simulation mode or real-world experiment mode
        exp_mode = 'realworld'      # it is either simulation or realworld
        
        
        #path = ReferencePath(self.interface.ref_path_msg)
        
        
        self.init_predicted_tensors(self.interface.obstacle_predictions, exp_mode)
        state_coordinates, ros_predicted_cov, prediction_dict = self.reformat_obstacles_odom(self.interface.obstacle_predictions, exp_mode)

        x = state_coordinates[:, 0]
        y = state_coordinates[:, 1]
        headings = torch.zeros_like(x)
        cov = torch.eye(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32).repeat(self.cfg["obstacles"]["num_obstacles"], 1, 1) * self.cfg["obstacles"]["initial_covariance"]
        vx = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
        vy = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
        obstacles_new = DynamicObstacles(self.cfg, x, y, headings, cov, vx, vy)
        
        

        ros_mode_probabilities = torch.tensor([1], device=self.cfg["device"])
        obstacles_new.set_mode_probabilities(ros_predicted_cov, ros_mode_probabilities)
        # These mode probabilities are based on the multi-modal experiment in Oscar's paper

        step_num = 0
        computational_time = []
        prediction_computation_time = []

        first_timestep = True
        max_cp = 1e-6
        collision_detected = False

        num_experiments = 100


        data_dict = {
            "collisions": [],
            "cp": [],
            "max_cp": [],
            "planning_time": [],
            "travelled_distance": [],
            "velocity": [],
            "duration": []
        }

        while not rospy.is_shutdown():

            robot_vel_vec = []
            
            if self.interface.spline is not None:
                if first_timestep:
                    gc.collect()

                    state_coordinates, ros_predicted_cov, prediction_dict = self.reformat_obstacles_odom(
                        self.interface.obstacle_predictions, exp_mode)

                    x = state_coordinates[:, 0]
                    y = state_coordinates[:, 1]
                    headings = torch.zeros_like(x)
                    cov = torch.eye(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"],
                                    dtype=torch.float32).repeat(self.cfg["obstacles"]["num_obstacles"], 1, 1) * \
                        self.cfg["obstacles"]["initial_covariance"]
                    vx = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
                    vy = torch.zeros(self.cfg["obstacles"]["num_obstacles"], device=self.cfg["device"], dtype=torch.float32)
                    obstacles_new = DynamicObstacles(self.cfg, x, y, headings, cov, vx, vy)

                    objective = DinovaObjective(self.cfg, plot=False, dyn_obstacles=obstacles_new, spline=self.interface.spline)
                    objective.update_goal([4., 2.0])

                    dynamics  = DinovaDynamics(self.cfg)
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
                    obstacles_new.set_mode_probabilities(ros_predicted_cov, ros_mode_probabilities)
                # print("odom mesage is ", self.interface.odom_msg)
                if self.interface.odom_msg is not None:
                    # orientation_quaternion = self.interface.odom_msg.pose.pose.orientation
                    # orientation_euler = euler_from_quaternion([
                    #     orientation_quaternion.x,
                    #     orientation_quaternion.y,
                    #     orientation_quaternion.z,
                    #     orientation_quaternion.w
                    # ])
                    # Extract individual Euler angles
                    # roll, pitch, yaw = orientation_euler
                    yaw = self.interface.odom_msg.position[2]
                    robot_position = [self.interface.odom_msg.position[0],
                                    self.interface.odom_msg.position[1],
                                    self.interface.odom_msg.position[2], 0.0]
                    robot_velocity = [self.interface.odom_msg.velocity[0],
                                    self.interface.odom_msg.velocity[1],
                                    self.interface.odom_msg.velocity[2]]
                    
                    print("robot x-position is: ", self.interface.odom_msg.position[0])
                    print("robot y-position is: ", self.interface.odom_msg.position[1])
                    
                    positions = torch.stack([torch.tensor(robot_position[0], device='cuda:0'), torch.tensor(robot_position[1], device='cuda:0')])
                    goal_dist = torch.linalg.norm(positions - objective.nav_goal) 
                    print("goal_dist is: ", goal_dist) 
                    print("goal is: ", objective.nav_goal[0])           
                    # if objective.nav_goal[0] < 0 and (robot_position[0] < -3.0 or robot_position[1] < -2.5):
                    #     objective.update_goal([4.0, 2.8])

                    #     self.rotating_to_goal = True
                    # elif objective.nav_goal[0] > 0 and (robot_position[0] > 3.8 or robot_position[1] > 2.6):
                    #     objective.update_goal([-3, -2.5])
                    #     self.rotating_to_goal = True

                    # if self.rotating_to_goal:
                    #     self.rotate_to_goal(objective.nav_goal, positions, yaw)
                    # else:
                        #print("navigation goal is: ", objective.nav_goal[0])

                        # Here need to update states of obstacles. 
                        # Can update with obstacles.update_states(states)
                        # Then also need the predicted states.
                        # Or can use the following code to make predictions based on the current position and velocity of the obstacles
                        #print("reformat obstacle odom")
                    state_coordinates, ros_predicted_cov, prediction_dict = self.reformat_obstacles_odom(self.interface.obstacle_predictions, exp_mode)
                    obstacles_new.state_coordinates = state_coordinates
                    #print("predicted dict x coordinate: ", self.prediction_dict)

                    obstacles_new.update_multi_mode_predicted_states(prediction_dict)
                


                    state = self.reformat_odom(robot_position, robot_velocity)
                    action_seq, all_samples, all_samples_cost = planner.command(state)

                    planning_time_stop = time.perf_counter()
                    
                    action = action_seq[0]

                    # publish planned trajectory
                    plan = dynamics.get_trajectory(state, action_seq)
                    # for action_u in action_seq:
                    #     print("planned v is: ", action_u[0].item(),  action_u[1].item())
                    #     print("planned w is: ", action_u[2].item())
                    '''
                    for i, point in enumerate(plan):
                        x, y = point[0].item(), point[1].item()
                        print("planned x is: ", x)
                        print("planned y is: ", y)
                    '''

                    self.interface.publish_planned_trajectory(plan)
                    self.interface.plot_mppi_rollouts(all_samples, all_samples_cost)


                    
                    step_num += 1

                    # actuate robot and sleep
                    self.interface.actuate(action)

                    

                    obstacles_predicted_states = []
                    for obst_index, obst_data in self.prediction_dict.items():
                        obstacles_predicted_states.append(obst_data["predicted_coordinates"])

                    obstacles_predicted_states = torch.stack((obstacles_predicted_states), dim=0)
                    time_step = 0

                    theta = robot_position[2]
                    
                

if __name__ == '__main__':
    try:
        planner = Planner()
        planner.run_dinova_robot()
    except rospy.ROSInterruptException:
        pass