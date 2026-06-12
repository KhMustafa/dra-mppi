import torch
from torch.distributions import MultivariateNormal
import numpy as np

# Monte Carlo Sampling
N_monte_carlo = 100
upper_x, lower_x = 0.6, -0.6
upper_y, lower_y = 0.6, -0.6

samples_x = torch.rand((N_monte_carlo), dtype=torch.float32) * (upper_x - lower_x) + lower_x
samples_y = torch.rand((N_monte_carlo), dtype=torch.float32) * (upper_y - lower_y) + lower_y
samples = torch.stack((samples_x, samples_y), dim=1)

# Mean and covariance matrix (3 batched Gaussians)
mean = torch.tensor([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]], dtype=torch.float32)
cov_matrix = torch.tensor([[[0.1, 0.0], [0.0, 0.1]]] * 3, dtype=torch.float32)

# Define multivariate normal distribution
dist = MultivariateNormal(mean, cov_matrix)

# Compute log probabilities
expanded_points = samples.unsqueeze(1).expand(-1, dist.batch_shape[0], -1)
log_probs = dist.log_prob(expanded_points)
pdf_values = torch.exp(log_probs).sum(dim=1)  # Sum over batch dimension

# --- Correct Normalization (using total integral) ---
# Normalize by total sum of the PDFs to make sure it integrates to 1
square_area = (upper_x - lower_x) * (upper_y - lower_y)
pdf_normalization_factor = pdf_values.sum() / square_area  # Normalize sum of densities
normalized_pdf_values = pdf_values / pdf_normalization_factor  # Proper probability density

# --- Simple Averaging (incorrect normalization) ---
# Normalize by dividing by the number of Gaussians (simple but incorrect)
simple_avg_pdf_values = pdf_values / dist.batch_shape[0]  # Divide by number of Gaussians

# Estimate the integral using the Monte Carlo method for both approaches
integral_radius = 0.6
within_bounds = ((samples[:, 0, None]) ** 2 + (samples[:, 1, None]) ** 2 <= integral_radius ** 2)

# Apply indicator function to keep values inside the circle
masked_values_normalized = normalized_pdf_values[:, None] * within_bounds
masked_values_simple_avg = simple_avg_pdf_values[:, None] * within_bounds

# Compute the column sums for both normalization methods
column_sums_normalized = torch.sum(masked_values_normalized, dim=0)
column_sums_simple_avg = torch.sum(masked_values_simple_avg, dim=0)

# Count how many samples are within the circle
true_counts = torch.sum(within_bounds, dim=0)

# Compute the mean values inside the circle
means_within_bounds_normalized = torch.where(true_counts > 0, column_sums_normalized / true_counts, 0.0)
means_within_bounds_simple_avg = torch.where(true_counts > 0, column_sums_simple_avg / true_counts, 0.0)

# Estimate the integral using the normalized PDF
circle_area = np.pi * integral_radius ** 2
integral_estimate_normalized = means_within_bounds_normalized * circle_area  # Correct estimate
integral_estimate_simple_avg = means_within_bounds_simple_avg * circle_area  # Approximate estimate

# Results
print(f"Normalized Integral Estimate (correct): {integral_estimate_normalized.item()}")
print(f"Simple Averaging Integral Estimate (approx): {integral_estimate_simple_avg.item()}")
