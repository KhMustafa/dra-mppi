import torch
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__)))
                
from cost_functions import ObjectiveFunctionsClass
    
class JackalObjective(ObjectiveFunctionsClass):

    def __init__(self, cfg, stat_obstacles=None, dyn_obstacles=None, goal=None, plot=False, spline=None):
        
        super().__init__(cfg, stat_obstacles, dyn_obstacles, goal)

        self.dyn_cost = torch.zeros(cfg["mppi"]["num_samples"], device=cfg["device"])
        self.current_dyn_cost = None
        self.dyn_weight = 200.0

        self.start_x = self.cfg["start"][0]
        self.start_y = self.cfg["start"][1]

        self.counter = 0
        self.spline = spline
        
        s_vals = spline.s
        s_start, s_end = s_vals[0], s_vals[-1]
        xs = torch.linspace(s_start, s_end, 100)

        self.points = [spline.calculate_position(s) for s in xs]  # list of (x, y) tuples
        self.points = torch.tensor(self.points, dtype=xs.dtype, device=cfg["device"])

        self.ref_path_tangents = [spline.deriv_normalized(s) for s in xs]
        self.ref_path_tangents = torch.tensor(self.ref_path_tangents, dtype=xs.dtype, device=cfg["device"])


        
        self.terminal_angle_weight  = 100
        self.terminal_contouring_mp = 10
        self.contour_weight = 10
        self.lag_weight = 0.0


        # Set maximum acceptable collision probability
        self.hard_cp_constraint = cfg["obstacles"]["hard_cp_constraint"]
        self.soft_cp_constraint = cfg["obstacles"]["soft_cp_constraint"]


    def compute_running_cost(self, state: torch.Tensor, t: int, action: torch.Tensor=None, **kwargs):
        if state.ndim == 2:
            self.x = state[:, 0]
            self.y = state[:, 1]
            self.heading = state[:, 2]
            self.v = state[:, 3]
            self.s = state[:, 4]
            self.yaw_rate = state[:, 5]
        elif state.ndim == 3:
            # compute cost along the horizon at once
            self.x = state[:, :, 0]
            self.y = state[:, :, 1]
            self.heading = state[:, :, 2]
            self.v = state[:, :, 3]
            self.s = state[:, :, 4]
            self.yaw_rate = state[:, :, 5]

        
        self.action = action
        self.goal_cost = self.calculate_goal_cost() * 5.0
        total_cost = torch.zeros_like(self.goal_cost)
        
        total_cost += self.calculate_goal_cost() * 5.0
        # total_cost += self.reverse_cost() * 1.0

        total_cost += self.line_follow_cost() * 5.0 # 0.5
        total_cost += self.calculate_tracking_velocity_cost() * 2.2 # 2.2, 4.0
        # total_cost += self.lane_boundary_cost()
        # total_cost += self.jackal_rotation_cost() * 10


        obstacle_mean_list = []
        obstacle_covs_list = []
        if state.ndim == 2:
            if self.dyn_obstacles is not None:
                for obst_index in range(self.dyn_obstacles.n_obstacles):
                    means = self.dyn_obstacles.multi_mode_predicted_coordinates[obst_index]["predicted_coordinates"][t, :, :]
                    covs  = self.dyn_obstacles.multi_mode_predicted_coordinates[obst_index]["predicted_covs"][t, :, :]
                    obstacle_mean_list.append(means)
                    obstacle_covs_list.append(covs)
                obstacle_means = torch.cat(obstacle_mean_list, dim=0)
                obstacle_covs  = torch.cat(obstacle_covs_list, dim=0)
                # print("obstacle_means.shape is: ", obstacle_means.shape)
                total_cost += self.calculate_bound_dyn_cost(obstacle_means, obstacle_covs, t)
        elif state.ndim == 3:
            # compute the cost over the horizon at once
            if self.dyn_obstacles is not None:
                for obst_index in range(self.dyn_obstacles.n_obstacles):
                    means = self.dyn_obstacles.multi_mode_predicted_coordinates[obst_index]["predicted_coordinates"]
                    covs  = self.dyn_obstacles.multi_mode_predicted_coordinates[obst_index]["predicted_covs"]
                    # print("covss shape is: ", covs.shape)
                    obstacle_mean_list.append(means)
                    obstacle_covs_list.append(covs)
                obstacle_means = torch.stack(obstacle_mean_list, dim=0).squeeze(2)
                obstacle_means = obstacle_means.permute(1,0,2)
                obstacle_covs = torch.stack(obstacle_covs_list, dim=0).squeeze(2)
                obstacle_covs = obstacle_covs.permute(1,0,2,3)
                
                total_cost += self.calculate_bound_dyn_cost(obstacle_means, obstacle_covs)

        return total_cost
    
    def haar_difference_without_abs(self, angle1, angle2):
            return torch.fmod(angle1 - angle2 + torch.pi, 2 * torch.pi) - torch.pi


    def calculate_bound_dyn_cost(self, obstacle_modes: torch.Tensor, obstacle_covs:  torch.Tensor, t=None):


        if obstacle_modes.ndim == 2:
            coll_prob = self.calculate_multi_modal_dynamic_obstacle_cost(obstacle_modes[:, 0], obstacle_modes[:, 1], obstacle_covs, t)
        else:
            # compute along the horizon at once
            coll_prob = self.calculate_multi_modal_dynamic_obstacle_cost(obstacle_modes[:, :, 0], obstacle_modes[:, :, 1], obstacle_covs)    
        
        dyn_cost = torch.where(coll_prob > self.hard_cp_constraint, 20000., 0) 
        dyn_cost += coll_prob * 100
        # self.dyn_cost += dyn_cost
            
        
        return dyn_cost

    def get_current_dyn_cost(self, cost: torch.Tensor):
        
        # If the collision probability is calculated, given the observation from the simulatrion, save it
        self.current_dyn_cost = cost.repeat(self.cfg["mppi"]["num_samples"])
    
    
    def calculate_tracking_velocity_cost(self):

        v_desired = 1.0
        if self.action.ndim == 2:
            velocity_cost = (self.action[:,0] - v_desired) ** 2
        elif self.action.ndim == 3:
            velocity_cost = (self.action[:,:,0] - v_desired) ** 2

        return velocity_cost
    
    def reverse_cost(self):

        if self.action.ndim == 2:
            reverse_cost = torch.where(self.action[:, 0] < 0, 1.0, 0.0)
        elif self.action.ndim == 3:
            reverse_cost = torch.where(self.action[:, :, 0] < 0, 1.0, 0.0)
        
        return reverse_cost
    
    def line_follow_cost(self):
        if self.action.ndim == 2:
            robot_positions = torch.stack((self.x, self.y), dim=1)
            diffs = self.points.unsqueeze(0) - robot_positions.unsqueeze(1)                 # (B, N, 2)
            dists = torch.norm(diffs, dim=2)                                                # (B, N)
            
            idx_min = torch.argmin(dists, dim=1) # (B,) 
            closest_points = self.points[idx_min]
            lateral_diffs = robot_positions - closest_points
            lateral_dists = torch.norm(lateral_diffs, dim=1)
        elif self.action.ndim == 3:  # (B, T, 2)
            # robot positions (B, T, 2)
            robot_positions = torch.stack((self.x, self.y), dim=2)  

            # diffs to reference points (B, T, N, 2)
            diffs = self.points.unsqueeze(0).unsqueeze(0) - robot_positions.unsqueeze(2)  
            dists = torch.norm(diffs, dim=3)  # (B, T, N)

            # closest point indices per (B, T)
            idx_min = torch.argmin(dists, dim=2)  # (B, T)

            # gather closest points (B, T, 2)
            closest_points = self.points[idx_min]  

            # lateral distance (B, T)
            lateral_diffs = robot_positions - closest_points
            lateral_dists = torch.norm(lateral_diffs, dim=2)

        return lateral_dists 
    
    def lane_boundary_cost(self):
        lane_boundary_cost = torch.where((self.y > 3.2) | (self.y < -2.9) | (self.x > 4.4) | (self.x < -3.4), 10000.0, 0.0)
        return lane_boundary_cost


    def jackal_rotation_cost(self):

        return torch.abs(self.yaw_rate)
    
    def contouring_cost(self, t):
        path_x, path_y = self.spline.at(self.s)
        path_dx_normalized, path_dy_normalized = self.spline.deriv_normalized(self.s)


        contour_error = path_dy_normalized * (self.x - path_x) - path_dx_normalized * (self.y - path_y)
        lag_error     = path_dx_normalized * (self.x - path_x) + path_dy_normalized * (self.y - path_y)
        
        lag_cost = self.lag_weight * lag_error**2
        contour_cost = self.contour_weight * contour_error**2
        cost = lag_cost + contour_cost

        if t == (self.cfg["mppi"]["horizon"] - 1):
            # Compute the angle w.r.t. the path
            path_angle = torch.atan2(path_dy_normalized, path_dx_normalized)
            angle_error = self.haar_difference_without_abs(self.heading, path_angle)

            cost += self.terminal_angle_weight  * angle_error**2
            cost += self.terminal_contouring_mp * self.lag_weight * lag_error**2
            cost += self.terminal_contouring_mp * self.contour_weight * contour_error**2
            
        return cost
            