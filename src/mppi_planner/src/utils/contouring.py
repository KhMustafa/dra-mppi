#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from scipy.interpolate import CubicSpline
import numpy as np
import torch
import time

class SplineFitter:
    def __init__(self):
        self._splines = None
        self._num_segments = 5
        self._closest_s = None
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


    def fit_path(self, path_msg):
        if not path_msg.poses:
            rospy.logwarn("Received empty path")
            return

        x = []
        y = []
        for pose in path_msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)

        if len(x) < 2:
            rospy.logwarn("Not enough points to fit splines")
            return

        self.fit_splines(x, y)


    def fit_splines(self, x, y):
        # Compute cumulative path lengths
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        s = np.cumsum(distances)
        s = np.insert(s, 0, 0)  # Include the starting point

        # Fit cubic splines to the path
        cs_x = CubicSpline(s, x)
        cs_y = CubicSpline(s, y)

        # Store splines and their parameters
        newsplines = []
        for i in range(len(s) - 1):
            spline = [
                cs_x.c[:, i][0], cs_x.c[:, i][1], cs_x.c[:, i][2], cs_x.c[:, i][3],
                cs_y.c[:, i][0], cs_y.c[:, i][1], cs_y.c[:, i][2], cs_y.c[:, i][3],
                s[i]
            ]
            newsplines.append(spline)
        self._splines = torch.tensor(newsplines, dtype=torch.float32, device=self.device)



    def ready(self):
        return self._splines is not None and self._splines.numel() > 0

    def get_active_splines(self, pos):
        
        pos_tensor = torch.tensor(pos, dtype=torch.float32, device=self.device)

        self._closest_s = self.find_closest_s(pos_tensor)
        self._closest_x, self._closest_y, index = self.evaluate_and_get_spline_index(self._closest_s)

        # Ensure index is a Python integer for slicing
        index = int(index)

        # Select the active splines
        max_index = min(index + self._num_segments, len(self._splines))
        active_splines = self._splines[index:max_index]

        return active_splines       

    def evaluate(self, s):
        x, y, _ = self.evaluate_and_get_spline_index(s)
        return x, y

    def evaluate_and_get_spline_index(self, s):
        if self._splines is None:
            rospy.logwarn("Splines have not been computed yet")
            return None, None, None
        
        # Extract s values from tensor
        s_values = self._splines[:, -1]  # Last column contains s values
        spline_index = torch.searchsorted(s_values.contiguous(), s, right=True).item() - 1
        spline_index = max(0, min(spline_index, len(self._splines) - 1))

        spline = self._splines[spline_index]
                
        # Compute t
        t = s - spline[-1]

        # Evaluate spline
        x = spline[0] * t**3 + spline[1] * t**2 + spline[2] * t + spline[3]
        y = spline[4] * t**3 + spline[5] * t**2 + spline[6] * t + spline[7]

        return x, y, spline_index
    
    def deriv(self, s):
        spline = self._splines[0]
        for k in range(5 - 1, 0, -1):
            value = 1
            
        return 3 * spline[0] * s * s + 2 * spline[1] * s + spline[2]


    def find_closest_s(self, point_tensor, tol=1e-5):
        if self._splines is None:
            rospy.logwarn("Splines have not been computed yet")
            return None
        
        # Total path length
        s_start = self._splines[0,  -1]
        s_end   = self._splines[-1, -1] + torch.sqrt(self._splines[-1, 0]**2 + self._splines[-1, 4]**2)

        # Bisection method
        while (s_end - s_start) > tol:
            s_mid1 = s_start + (s_end - s_start) / 3
            s_mid2 = s_end   - (s_end - s_start) / 3

            x_mid_1, y_mid_1 = self.evaluate(s_mid1)
            x_mid_2, y_mid_2 = self.evaluate(s_mid2)


            d_mid1 = torch.norm(torch.stack([x_mid_1, y_mid_1]) - point_tensor)
            d_mid2 = torch.norm(torch.stack([x_mid_2, y_mid_2]) - point_tensor)

            if d_mid1 < d_mid2:
                s_end = s_mid2
            else:
                s_start = s_mid1

        s_closest = (s_start + s_end) / 2
        return s_closest.item()


def main():
    # Initialize the spline fitter
    fitter = SplineFitter()
    
    # Generate a long synthetic path (e.g., 10,000 points forming a smooth curve)
    num_points = 30
    x = np.linspace(0, 100, num_points)
    y = np.sin(x) * 10  # Example: a sine wave path

    # Fit the splines
    fitter.fit_splines(x, y)

    # Generate a random query point near the path
    query_point = np.array([50.0, 0.0])  # Midway through the path

    # Move data to CUDA
    query_tensor = torch.tensor(query_point, dtype=torch.float32, device=fitter.device)

    # Measure time for CUDA-accelerated `find_closest_s`
    torch.cuda.synchronize()  # Ensure accurate timing
    start_time = time.time()
    closest_s_cuda = fitter.find_closest_s(query_tensor)
    torch.cuda.synchronize()  # Ensure all GPU operations complete before timing
    cuda_time = time.time() - start_time

    # Measure time for a NumPy-based version (CPU)
    cpu_fitter = SplineFitter()  # Another instance for CPU comparison
    cpu_fitter.fit_splines(x, y)

    start_time = time.time()
    closest_s_cpu = cpu_fitter.find_closest_s(query_point)
    cpu_time = time.time() - start_time

    # Print results
    print(f"CUDA Closest s: {closest_s_cuda:.6f}, Time: {cuda_time:.6f} sec")
    print(f"CPU Closest s: {closest_s_cpu:.6f}, Time: {cpu_time:.6f} sec")
    print(f"Speedup Factor: {cpu_time / cuda_time:.2f}x")

if __name__ == "__main__":
    main()

