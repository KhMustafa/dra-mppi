import torch
import numpy as np
from torch.distributions import MultivariateNormal
from scipy.stats import multivariate_normal

# Define mean and covariance matrix
mean = np.array([1.0, 2.0])  # Mean for the Gaussian distribution
cov_matrix = np.array([[0.1, 0.0], [0.0, 0.1]])  # Covariance matrix for the Gaussian distribution

# Define the region (circle with radius r and center (x_0, y_0))
x0, y0 = 0, 0
radius = 0.6

# Define the Multivariate Normal distribution
distribution = multivariate_normal(mean=mean, cov=cov_matrix)

# To compute the exact integral, we can approximate it using the CDF of the multivariate Gaussian over the region (circle)
def compute_exact_integral(x0, y0, r, distribution):
    # We'll integrate over the region defined by the circle of radius r centered at (x0, y0)
    # This involves calculating the cumulative probability within the circle.
    
    # Create a grid of points within the bounding box of the circle
    num_points = 1000  # Adjust the resolution as needed
    x_vals = np.linspace(x0 - r, x0 + r, num_points)
    y_vals = np.linspace(y0 - r, y0 + r, num_points)

    # Set up a grid to evaluate the PDF
    grid_x, grid_y = np.meshgrid(x_vals, y_vals)
    points = np.vstack([grid_x.ravel(), grid_y.ravel()]).T

    # Mask the points that are outside the circle
    mask = (points[:, 0] - x0)**2 + (points[:, 1] - y0)**2 <= r**2
    inside_points = points[mask]
    
    # Calculate the PDF values for the points inside the circle
    pdf_values = distribution.pdf(inside_points)
    
    # Approximate the integral by summing the PDF values and multiplying by the area of each point
    area_element = (2 * r / num_points) ** 2  # Area of each point (approximation)
    integral = np.sum(pdf_values) * area_element
    
    return integral

# Compute the exact integral over the region
exact_integral = compute_exact_integral(x0, y0, radius, distribution)
print(f"Exact integral estimate: {exact_integral}")