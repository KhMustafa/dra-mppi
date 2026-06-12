import torch
import numpy as np
from scipy.linalg import solve_continuous_are

class Priors(object):
    def __init__(self, cfg):
        self.device = cfg["device"]
        self.horizon = cfg["mppi"]["horizon"]
        # Set up cost matrices (keep them on CPU for the Riccati solver)
        self.Q = torch.diag(torch.tensor([10, 10, 1], dtype=torch.float64)).cpu().numpy()
        self.R = torch.diag(torch.tensor([1, 1], dtype=torch.float64)).cpu().numpy()

    def compute_priors(self, state: torch.Tensor, t: int, guidance_x: torch.Tensor, guidance_y: torch.Tensor):
        # Linearize system around the current state, state 0 because the first sample is the lqr controller!
        A = np.array([[0, 0, -torch.sin(state[0, 2]).item()],
                      [0, 0,  torch.cos(state[0, 2]).item()],
                      [0, 0, 0]], dtype=np.float64)
        B = np.array([[torch.cos(state[0, 2]).item(), 0],
                      [torch.sin(state[0, 2]).item(), 0],
                      [0, 1]], dtype=np.float64)
        
        # Compute LQR gain
        K = self.lqr_controller(A, B)
        # desired x, y
        x_d = guidance_x[t+1] 
        y_d = guidance_y[t+1]   
        # previous x, y    
        x_prev = guidance_x[t]
        y_prev = guidance_y[t] 
        # Desired orientation (heading) angle
        theta_d = torch.atan2(y_d - y_prev, x_d - x_prev)

        # Compute control input (v, omega) using LQR feedback
        error = torch.tensor([state[0, 0] - x_d, state[0, 1] - y_d, state[0, 2] - theta_d], dtype=torch.float64, device=self.device)
        u = -K @ error

        return u
    
    def lqr_controller(self, A, B):
        # Solve Riccati equation using SciPy
        P = solve_continuous_are(A, B, self.Q, self.R)
        # Compute LQR gain
        K = torch.tensor(np.linalg.inv(self.R) @ B.T @ P, device=self.device, dtype=torch.float64)
        return K
