import torch
from torch.distributions import MultivariateNormal
import numpy as np
# Sample a grid of points of shape (self.N_monte_carlo, 2) with x and y ranging from -5 to 5
N_monte_carlo = 100000
upper_x = 0.6
lower_x = -0.6
lower_y = -0.6

upper_y = 0.6
samples_x = torch.rand((N_monte_carlo), device = "cpu", dtype=torch.float32) * (
                         upper_x - lower_x) + lower_x

samples_y = torch.rand((N_monte_carlo), device = "cpu", dtype=torch.float32) * (
                        upper_y - lower_y) + lower_y

samples = torch.stack((samples_x, samples_y), dim=1)
# Mean and covariance matrix
mean = torch.tensor([
    [1.0, 2.0],   # First batch element
    [3.0, 4.0],   # Second batch element
    [5.0, 6.0]    # Third batch element
])

# Batched covariance: shape (3, 2, 2)
cov_matrix = torch.tensor([
    [[0.1, 0.0], [0.0, 0.1]],  # Covariance for first batch element
    [[0.1, 0.0], [0.0, 0.1]],  # Covariance for second batch element
    [[0.1, 0.0], [0.0, 0.1]]   # Covariance for third batch element
])

# Define distribution
dist = MultivariateNormal(mean, cov_matrix)
expanded_points = samples.unsqueeze(1).expand(-1, dist.batch_shape[0], -1)
log_probs = dist.log_prob(expanded_points)
probs   = torch.exp(log_probs)
sum_pdf = torch.exp(log_probs).sum(dim=1)  # TODO: Try this instead again.

# Normalize the PDF sum to ensure it integrates to 1
square_area = (upper_x - lower_x) * (upper_y - lower_y)
pdf_normalization_factor = sum_pdf.sum() / square_area  # Normalize sum of densities
normalized_pdf_values    = sum_pdf / pdf_normalization_factor  # Proper probability density


# Check which samples are within the specified bounds
x = 0
y = 0
integral_radius = 0.6
within_bounds = ((samples[:, 0, None] - x) ** 2 + (
                  samples[:, 1, None] - y) ** 2 <= integral_radius ** 2)

masked_values = normalized_pdf_values[:, None] * within_bounds
column_sums = torch.sum(masked_values, dim=0)
true_counts = torch.sum(within_bounds, dim=0)

means_within_bounds = torch.where(true_counts > 0, column_sums / true_counts, 0.0)
# The integral is approximated as the proportion of points within the bounds multiplied by the area of the rectangular region
area = np.pi * integral_radius ** 2
integral_estimate = (means_within_bounds) * area

print("approximate integral is: ", integral_estimate)