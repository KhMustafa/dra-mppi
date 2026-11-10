import time
import torch
import os
import numpy as np


current_dir = os.path.dirname(__file__)

class DynamicObstacles(object):

    def __init__(self, cfg, x, y, yaw, cov, vx, vy) -> None:

        # Set meta parameters
        self.print_time = cfg["obstacles"][
            "print_time"]  # Set to True to print the time it takes to perform certain operations
        self.use_batch_gaussian = cfg["obstacles"]["use_gaussian_batch"]  # True is faster

        # Save x, y and cov in the correct format
        if isinstance(x, int) or isinstance(x, float):
            x = torch.tensor([x], device=cfg["device"], dtype=torch.float32)

        if isinstance(y, int) or isinstance(y, float):
            y = torch.tensor([y], device=cfg["device"], dtype=torch.float32)

        if cov.ndim == 2:
            cov = cov.unsqueeze(0)

        if vx.ndim == 1:
            vx = vx.unsqueeze(0)
            vy = vy.unsqueeze(0)

        # Save the inputs as the actual current state of the obstacle
        self.cfg = cfg
        self.state_coordinates = torch.stack((x, y), dim=1)
        self.state_cov = cov
        self.vx = vx
        self.vy = vy
        self.yaws = yaw

        # Save mppi parameters
        # self.n_obstacles = cfg["obstacles"]["num_obstacles"] = len(x)
        
        # For realworld experiment
        self.n_obstacles = 3
        self.N_rollouts = cfg["mppi"]["num_samples"]
        self.t = cfg["mppi"]["horizon"]

        # Initialise the predicted states of the obstacle
        # Repeat the state_coordinates and state_cov for the entire time horizon
        self.predicted_coordinates = self.state_coordinates.repeat(cfg["mppi"]["horizon"], 1, 1)
        self.predicted_covs = self.state_cov.repeat(cfg["mppi"]["horizon"], 1, 1, 1)
        # print("self.predicted_covs: ", self.predicted_covs.shape)

        self.predicted_velocities = torch.zeros_like(self.predicted_coordinates)

        # Initialise the predicted states of the obstacle
        # Repeat the state_coordinates and state_cov for the entire time horizon
        self.multi_mode_predicted_coordinates = dict()
        self.multi_mode_predicted_covs = dict()
        for n_obst in range(self.n_obstacles):
            # DIMS: timesteps, modes, coordinates
            self.multi_mode_predicted_coordinates[n_obst] = torch.zeros(cfg["mppi"]["horizon"], 1, 2,
                                                                        device=cfg["device"])
            self.multi_mode_predicted_covs[n_obst] = torch.zeros(cfg["mppi"]["horizon"], 1, 2, 2, device=cfg["device"])

        # Set values used for monte carlo integration
        self.N_monte_carlo = cfg["obstacles"][
            "N_monte_carlo"]  # NOTE: Has large influence on the runtime of the cost calculation
        self.integral_radius = cfg["obstacles"]["integral_radius"]
        self.sample_bound = cfg["obstacles"]["sample_bound"]

        self.take_samples(torch.tensor(cfg["start"], device=cfg["device"]))


    def take_samples(self, coordinates):

        self.map_x0 = coordinates[0] - self.sample_bound
        self.map_x1 = coordinates[0] + self.sample_bound
        self.map_y0 = coordinates[1] - self.sample_bound
        self.map_y1 = coordinates[1] + self.sample_bound

        # Sample a grid of points of shape (self.N_monte_carlo, 2) with x and y ranging from -5 to 5
        samples_x = torch.rand((self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32) * (
                    self.map_x1 - self.map_x0) + self.map_x0
        samples_y = torch.rand((self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32) * (
                    self.map_y1 - self.map_y0) + self.map_y0
        self.samples = torch.stack((samples_x, samples_y), dim=1)

    def take_samples_given_bounds(self, x, y):

        if x.ndim == 1:
            lower_x = torch.min(x) - self.cfg["obstacles"]["integral_radius"]
            upper_x = torch.max(x) + self.cfg["obstacles"]["integral_radius"]
            lower_y = torch.min(y) - self.cfg["obstacles"]["integral_radius"]
            upper_y = torch.max(y) + self.cfg["obstacles"]["integral_radius"]

            # Sample a grid of points of shape (self.N_monte_carlo, 2) with x and y ranging from -5 to 5
            samples_x = torch.rand((self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32) * (
                        upper_x - lower_x) + lower_x
            samples_y = torch.rand((self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32) * (
                        upper_y - lower_y) + lower_y
            self.samples = torch.stack((samples_x, samples_y), dim=1)
        elif x.ndim == 2:
            # compute cost at once along the horizon
            lower_x = torch.min(x, dim=0).values - self.cfg["obstacles"]["integral_radius"]
            upper_x = torch.max(x, dim=0).values + self.cfg["obstacles"]["integral_radius"]
            lower_y = torch.min(y, dim=0).values - self.cfg["obstacles"]["integral_radius"]
            upper_y = torch.max(y, dim=0).values + self.cfg["obstacles"]["integral_radius"]

            # print("lowe_x.shape is: ", lower_x.shape)
            
            # Sample [N, N_monte_carlo] values uniformly within the bounds
            rand_x = torch.rand((x.shape[1], self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32)
            rand_y = torch.rand((x.shape[1], self.N_monte_carlo), device=self.cfg["device"], dtype=torch.float32)

            # Rescale to per-timestep bounds
            samples_x = rand_x * (upper_x - lower_x).unsqueeze(1) + lower_x.unsqueeze(1)  # shape: [50, N]
            samples_y = rand_y * (upper_y - lower_y).unsqueeze(1) + lower_y.unsqueeze(1)  # shape: [50, N]
            # print("sampe_x.shape is: ", samples_x.shape)

            self.samples = torch.stack((samples_x, samples_y), dim=2)    # shape: [50, N_monte_carlo, 2]

    def update_states(self, state):

        self.state_coordinates = state[:2].unsqueeze(0)
        self.yaws = state[2]
        self.vx = state[3]
        self.vy = state[4]

    ############## FOR MULTI-MODAL PREDICTIONS ##############
    def set_mode_probabilities(self, covs, mode_probabilities):
        # print("set mode probabilities")
        self.predicted_covs = covs
        self.mode_probabilities = mode_probabilities

    def get_multi_mode_probabilities(self, covs, mode_probabilities):
        for n_obst in range(self.n_obstacles):
            self.multi_mode_predicted_covs[n_obst] = covs[n_obst]
        self.mode_probabilities = mode_probabilities

    def update_multi_mode_predicted_states(self, coordinates):
        for n_obst in range(self.n_obstacles):
            self.multi_mode_predicted_coordinates[n_obst] = coordinates[n_obst]
            self.multi_mode_predicted_covs[n_obst] = coordinates[n_obst]
          
    def create_multi_modal_gaussians(self, x, y, obstacle_covs=None, t=-1):

        # Save x, y and cov in the correct format
        if isinstance(x, int) or isinstance(x, float):
            x = torch.tensor([x], device=self.cfg["device"], dtype=torch.float32)

        if isinstance(y, int) or isinstance(y, float):
            y = torch.tensor([y], device=self.cfg["device"], dtype=torch.float32)

        if x.ndim == 0:
            x = x.unsqueeze(0)
            y = y.unsqueeze(0)

        # Save x, y and cov as float32 tensors
        # x, y and cov are already tensors but only the data type has to be changed to float32
        x = x.to(dtype=torch.float32)
        y = y.to(dtype=torch.float32)

        # Set initial values for the gaussian
        if x.ndim == 1:
            self.coordinates = torch.stack((x, y), dim=1)
        else:
            self.coordinates = torch.stack((x, y), dim=2)

        # print("predicted_cov shape: ", self.predicted_covs.shape)
        # print("obstacle_covs: ", obstacle_covs)
        if obstacle_covs is not None:
            cov = obstacle_covs
        elif self.predicted_covs.ndim == 3:
            cov = self.predicted_covs[t]
        
        # print("cov: ", cov.shape)

        # Create only batch Gaussian
        self.update_gaussian_batch(self.coordinates, cov)


    # Function that creates a single batch of torch Gaussians
    def update_gaussian_batch(self, coordinates, cov):
        '''
        print("torch version is: ", torch.__version__)

        print(coordinates)
        print("coordinates dims: ", coordinates.shape)
        print(cov)
        print("cov dimension", cov.shape)
        '''

        # Create Gaussian batch used by the torch and Monte Carlo method
        t = time.time()
        # TODO: Use scale_tril instead of cov to initialise the Gaussian faster
        # print("cov shape is: ", cov.shape)
        if cov.ndim == 2:
            self.torch_gaussian_batch = torch.distributions.multivariate_normal.MultivariateNormal(coordinates,
                                                                                                   cov ** 2)  # THIS WAS DUMB MISTAKE, COV SHOULD BE SQUARED
        else:
            self.torch_gaussian_batch = torch.distributions.multivariate_normal.MultivariateNormal(coordinates,
                                                                                                   cov ** 2)  # Here use a full covariance matrix calculated for multi-modal
        if self.print_time:
            print(f"Time to create torch gaussians batch: {time.time() - t}")

    def monte_carlo_sample_batch(self, posterior=False):

        t = time.time()

        # Expand points to match the batch size and compute log_prob
        # print("self.samples.shape is: ", self.samples.shape)
        if self.samples.ndim == 2:
            expanded_points = self.samples.unsqueeze(1).expand(-1, self.torch_gaussian_batch.batch_shape[0], -1)
        elif self.samples.ndim == 3:
            expanded_points = self.samples.permute(1,0,2).unsqueeze(2).expand(-1, -1, self.torch_gaussian_batch.batch_shape[1], -1)

        
        # print("self.torch_gaussian_batch.shape: ", self.torch_gaussian_batch.shape)
        # print("expanded_points.shape: ", expanded_points.shape)
        log_probs = self.torch_gaussian_batch.log_prob(expanded_points)
        probs = torch.exp(log_probs)
    
        # If there are more modes than obstacles: multi-modal predictions
        if probs.shape[1] > self.n_obstacles and probs.ndim == 2:

            # Save the collision probabilities per obstacle
            probs_per_obstacle = torch.zeros((self.N_monte_carlo, self.cfg["obstacles"]["num_obstacles"]),
                                                device=self.cfg["mppi"]["device"])

            # Loop through the obstacles
            start_index = 0
            for n_obst in range(self.n_obstacles):

                #if self.multi_mode_predicted_coordinates[n_obst]["crossed"] == True:
                if self.multi_mode_predicted_coordinates[n_obst]["predicted_coordinates"].shape[1] > 1:
                    #print("multi modes")
                    n_modes = self.mode_probabilities.shape[0]
                    obstacle_prob = (probs[:, start_index:start_index + n_modes] * self.mode_probabilities[:, n_obst]).sum(dim=1)
                    probs_per_obstacle[:, n_obst] = obstacle_prob
                    start_index += n_modes

            probs = probs_per_obstacle

        
        #print(log_probs.shape)
        if posterior == True:
            self.sum_pdf = 1 - ((1 - probs).prod(dim=probs.ndim - 1))
        else:
            #self.sum_pdf = torch.exp(log_probs).sum(dim=1)  # TODO: Try this instead again.
            #self.sum_pdf = probs.max(dim=1).values  # Take the max rather than sum over all obstacles
            self.sum_pdf = 1 - ((1 - probs).prod(dim=probs.ndim - 1))
            

        if self.print_time:
            print(f"Time to calculate pdf values batch: {time.time() - t}")

    ########## MONTE CARLO VERSION ##########
    # Create a function that does the same as integrate_one_shot_monte_carlo but it takes in x and y which are the centers of circles
    # The within bounds check has to be done on all samples if they are within a circle with radius r
    def integrate_one_shot_monte_carlo_circles(self, x, y, t=None, posterior=False):

        # Take samples
        self.take_samples_given_bounds(x, y)

        # Sample the torch Gaussian for Monte Carlo integration
        self.monte_carlo_sample_batch(posterior)

        # # Create the tensors for x, y and r
        if not isinstance(x, torch.Tensor):
            x, y = map(lambda v: torch.as_tensor(v, device=self.cfg["device"], dtype=torch.float32), (x, y))


        # Check which samples are within the specified bounds        
        if x.ndim == 1 or x.ndim == 0:
            within_bounds = ((self.samples[:, 0, None] - x) ** 2 + (
                    self.samples[:, 1, None] - y) ** 2 <= self.integral_radius ** 2)
        elif x.ndim == 2:
            samples_permuted = self.samples.permute(1,0,2)    # shape (20000, 20, 2)
            samples_x = samples_permuted[:, :, 0]
            samples_y = samples_permuted[:, :, 1]
            samples_x_exp = samples_x.unsqueeze(1)  # [20000, 1, 20]
            samples_y_exp = samples_y.unsqueeze(1)  # [20000, 1, 20]
            x_exp = x.unsqueeze(0)                  # [1, 400, 20]
            y_exp = y.unsqueeze(0)                  # [1, 400, 20]

            dist_squared = (samples_x_exp - x_exp) ** 2 + (samples_y_exp - y_exp) ** 2  # [20000, 400, 20]
            within_bounds = (dist_squared <= self.integral_radius ** 2)  # [20000, 400, 20]
        
        # print("within bounds shape: ", within_bounds.shape)

        # Calculate the integrals per timestep in one go
        column_sums, true_counts = self.create_mask(within_bounds)

        means_within_bounds = torch.where(true_counts > 0,
                                           column_sums / true_counts,
                                             0.0)

        # The integral is approximated as the proportion of points within the bounds multiplied by the area of the rectangular region
        area = np.pi * self.integral_radius ** 2
        integral_estimate = (means_within_bounds) * area

        return integral_estimate


    def create_mask(self, within_bounds):
        # print("self.sum_pdf.shape is: ", self.sum_pdf.shape)
        if self.use_batch_gaussian:  # FASTER
            if self.sum_pdf.ndim == 1:
                masked_values = self.sum_pdf[:, None] * within_bounds
            elif self.sum_pdf.ndim == 2:
                masked_values = self.sum_pdf.unsqueeze(1) * within_bounds
        else:
            masked_values = self.pdf_values[:, None] * within_bounds

        column_sums = torch.sum(masked_values, dim=0)
        true_counts = torch.sum(within_bounds, dim=0)

        # print(column_sums)

        return column_sums, true_counts


if __name__ == "__main__":

    pass