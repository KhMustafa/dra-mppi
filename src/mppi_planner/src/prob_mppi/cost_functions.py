import torch

class ObjectiveFunctionsClass(object):

    def __init__(self, cfg, stat_obstacles=None, dyn_obstacles=None, goal=None):
        
        # Save config files
        self.cfg = cfg
        self.nav_goal = torch.tensor(cfg["goal"], device=cfg["device"], dtype=torch.float16)
        
        # Save the dynamic and static obstacles
        self.dyn_obstacles  = dyn_obstacles
        self.stat_obstacles = stat_obstacles

        self.compute = self.compute_running_cost

        self.v_reference = 1.0

    
    def update_goal(self, goal):

        self.nav_goal = torch.tensor(goal, device=self.cfg["device"], dtype=torch.float16)

    def calculate_reference_velocity_cost(self):

        # Calculate the cost of the reference velocity
        v = (self.vx**2 + self.vy**2)**0.5
        speed_cost = torch.abs(self.v_reference - v)
        speed_cost = speed_cost - torch.min(speed_cost)
        speed_cost = speed_cost / torch.max(speed_cost)

        # Replace all nan values
        speed_cost[torch.isnan(speed_cost)] = 0

        return speed_cost
    
    def calculate_goal_cost(self):
        # Calculate distance to the goal
        if self.x.ndim == 1:
            positions = torch.stack([self.x, self.y], dim=1)
            goal_dist = torch.linalg.norm(positions - self.nav_goal, axis=1)
            # goal_dist = goal_dist - torch.min(goal_dist)
            # goal_dist = goal_dist / torch.max(goal_dist)
        elif self.x.ndim == 2:
            # Stack to get positions shape: (400, 20, 2)
            positions = torch.stack([self.x, self.y], dim=2)
            goal_diff = positions - self.nav_goal
            
            # Calculate Euclidean distance along last dimension
            goal_dist = torch.linalg.norm(goal_diff, dim=2)  # shape: (400, 20)

        # Assuming goal_dist is your tensor with potential NaN values
        goal_dist = torch.nan_to_num(goal_dist, nan=0.0)
        return goal_dist
    
    
    def calculate_speed_limit_cost(self):
            
        # Calculate the cost of the velocity
        v = (self.vx**2 + self.vy**2)**0.5

        # Set value to 1 if v is greater than 0.1
        speed_cost = torch.where(v > self.max_speed, 1.0, 0.0)

        return speed_cost
    
    
    def calculate_deterministic_obstacle_cost(self, t=-1, **kwargs):

        obstacle_headings = self.dyn_obstacles.yaws

        if self.dyn_obstacles.predicted_coordinates.ndim == 2:
            # Turn (N, 2) into (N, 1, 2)
            self.dyn_obstacles.predicted_coordinates = self.dyn_obstacles.predicted_coordinates.unsqueeze(1)
            self.dyn_obstacles.predicted_velocities = self.dyn_obstacles.predicted_velocities.unsqueeze(1)
            self.dyn_obstacles.predicted_covs = self.dyn_obstacles.predicted_covs.unsqueeze(1)

        collision_detected_int = torch.zeros(self.cfg["mppi"]["num_samples"], device=self.cfg["device"])

        for ts_index in range(self.dyn_obstacles.predicted_coordinates.shape[1]):
            pred_coordinates = self.dyn_obstacles.predicted_coordinates[:, ts_index, :]

            # If cost is calculated for a single timestep
            if len(self.x) == self.cfg["mppi"]["num_samples"]:
                x_obstacle = pred_coordinates[t, 0]
                y_obstacle = pred_coordinates[t, 1]
                heading_obstacle = obstacle_headings[ts_index]

            else:
                pred_coordinates = pred_coordinates.repeat(len(self.x)//pred_coordinates.shape[0], 1)
                x_obstacle = pred_coordinates[:, 0]
                y_obstacle = pred_coordinates[:, 1]
                heading_obstacle = obstacle_headings[ts_index]

            x_os = self.x
            y_os = self.y
            heading_os = self.heading

            # Set the center distance
            center_distance = torch.tensor([-0.3, -0.1, 0.1, 0.3], device=self.cfg["device"])

            # Multiply x_direction and y_direction with the center_distance to get 4 (x, y) coordinates
            x_circles_obst = x_obstacle + torch.cos(heading_obstacle) * center_distance
            y_circles_obst = y_obstacle + torch.sin(heading_obstacle) * center_distance
            centres_obstacle = torch.stack([x_circles_obst, y_circles_obst], dim=1)

            x_circles_os = x_os.unsqueeze(-1) + torch.cos(heading_os).unsqueeze(-1) * center_distance
            y_circles_os = y_os.unsqueeze(-1) + torch.sin(heading_os).unsqueeze(-1) * center_distance
            centres_os = torch.stack([x_circles_os, y_circles_os], dim=2)
            centres_os_flat = centres_os.reshape(-1, 2)  # -1 will infer the correct size, resulting in (3200, 2)
            
            distances = torch.cdist(centres_os_flat, centres_obstacle)
            distances = distances.reshape(self.cfg["mppi"]["num_samples"], 4, 4)

            # Set the radius and threshold distance
            radius = 0.4
            threshold_distance = 2 * radius

            # Check for collisions
            collision_mask = distances < threshold_distance
            collision_detected = collision_mask.any(dim=1).any(dim=1)
            collision_detected_int += collision_detected.int()

        return collision_detected_int

    
    def calculate_multi_modal_dynamic_obstacle_cost(self, predicted_x, predicted_y, obstacle_covs, t=None):

        # Create Gaussians based on the predicted coordinates and covariances and evaluate their integrals and normalize between 0 and 1
        self.dyn_obstacles.create_multi_modal_gaussians(predicted_x, predicted_y, obstacle_covs, t=t)
        total_obstacle_cost = self.dyn_obstacles.integrate_one_shot_monte_carlo_circles(self.x, self.y, t=t, posterior=False)

        return total_obstacle_cost
    

    def normalize(self, cost):

        cost = cost - torch.min(cost)
        cost = cost / torch.max(cost)

        # Replace all nan values
        cost = torch.nan_to_num(cost, nan=0.0)

        return cost
  