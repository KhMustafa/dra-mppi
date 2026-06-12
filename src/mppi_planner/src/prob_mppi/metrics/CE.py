import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# Define possible intentions
theta_values = np.array([0.2, 0.8])  # Different levels of aggressiveness in lane change
belief = np.array([0.6, 0.4])  # Probabilities of each intention

# Define cost function
def cost_function(x, u, theta):
    """
    Compute cost based on distance and collision risk.
    """
    distance_cost = (10 - (x + u))**2  # Penalize deviation from goal (x = 10)
    collision_risk = np.exp(-5 * theta * (x + u))  # Higher theta means more risk
    return distance_cost + 10 * collision_risk  # Weighted sum

# Certainty-Equivalent Control (CEC): Optimize using mean(theta)
theta_mean = np.dot(belief, theta_values)  # Expected theta
u_range = np.linspace(-1, 2, 50)  # Possible control actions
costs_cec = [cost_function(5, u, theta_mean) for u in u_range]  # Evaluate costs

# Expectation-Based Optimization (EBO): Optimize expected cost over all θ
costs_ebo = [
    np.sum(belief * np.array([cost_function(5, u, theta) for theta in theta_values]))
    for u in u_range
]

# Plot results
plt.figure(figsize=(8, 5))
plt.plot(u_range, costs_cec, label="Certainty-Equivalent Control", linestyle="--")
plt.plot(u_range, costs_ebo, label="Expectation-Based Optimization", linestyle="-")
plt.xlabel("Control Input (u)")
plt.ylabel("Cost")
plt.legend()
plt.title("Comparison of Certainty-Equivalent vs. Expectation-Based Optimization")
plt.show()
