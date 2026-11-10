#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from derived_object_msgs.msg import Object, ObjectArray
from mpc_planner_msgs.msg import ObstacleArray
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
from robot_localization.srv import SetPose
import numpy as np
from scipy.interpolate import CubicSpline
# from guidance_planner.srv import guidances  # package not in workspace; guidance_service() is unused
import matplotlib.pyplot as plt  # For color mapping
from sensor_msgs.msg import Joy
import torch
from utils.spline import TorchSpline2D


class DynamicObstacle(object):
    def __init__(self, id, disc_start_id, radius):
        self.obstacle_id = id
        self.disc_start_id = disc_start_id
        self.radius = radius
        self.pose = None

class ReferencePath:
    def __init__(self, length=10):
        self.x = []
        self.y = []
        self.psi = []
        self.v = []
        self.s = []


class JackalInterface:
    def __init__(self, cfg):
        rospy.init_node('jackal_controller', anonymous=True)

        # velocity commands publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_cmd = Twist()

        # planned trajectory publisher
        self.plan_publisher = rospy.Publisher('/mppi_planned_trajectory', Path, queue_size=10)
        self.planned_trajectory = Path()

        self.rollouts_publisher = rospy.Publisher("/mppi_rollouts", MarkerArray, queue_size=10)


        # subscribe to robot's states
        rospy.Subscriber('/jackal1/odometry/filtered', Odometry, self.odom_callback)
        # rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.odom_msg = None
        self.spline = None

        # subscribe to pedestrians predictions
        
        # for simulated experiments
        '''
        rospy.Subscriber('/pedestrian_simulator/trajectory_predictions', ObstacleArray,
                         self.obstacle_predictions_callback)
        '''
        # for realworld experiments
        rospy.Subscriber('/vicon_util/dynamic_objects', ObjectArray,
                         self.obstacle_predictions_callback)
        
        
        
        self.obstacle_predictions_msg = None
        self.obstacle_predictions = None

        # subscribe to bluetooth joystick
        rospy.Subscriber('/bluetooth_teleop/joy', Joy, self.bluetooth_callback)

        # subscribe to reference path information
        rospy.Subscriber('/roadmap/reference', Path,
                         self.reference_path_callback)
        self.ref_path_msg = None

        self.enable_output = False

        # service clients for resetting
        
        self.reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_world', EmptySrv)
        self.reset_ekf_client = rospy.ServiceProxy('/set_pose', SetPose)
        self.reset_simulation_pub = rospy.Publisher('/lmpcc/reset_environment', EmptyMsg, queue_size=1)
        

        # Create a service proxy
        # self.guidance_client = rospy.ServiceProxy('/guidance_planner', guidances)

        # Set the rate at which to publish commands (in Hz)
        self.rate = rospy.Rate(10)  # 10 Hz
    
    ######################################### CALLBACK FUNCTIONS #########################################

    def odom_callback(self, msg):
        # store the received odometery message
        self.odom_msg = msg


    def obstacle_predictions_callback(self, obstacle_predictions_msg):
        # rospy.logwarn("pedestrians predictions are received")


        
        self.obstacle_predictions_msg = obstacle_predictions_msg
        # print("Obstacle prediction callback is called")
        # print(len(obstacle_predictions_msg.obstacles), " obstacle predictions received")
        if len(obstacle_predictions_msg.objects) == 0 or obstacle_predictions_msg.objects[0].id == -1:
            rospy.logwarn('Predictions received when obstacle positions are unknown, not loading.')
            return

        # load predictions into the obstacles
        self.obstacle_predictions = obstacle_predictions_msg
    
    def reference_path_callback(self, ref_path_msg):
        # Filter equal paths
        # if self.ref_path_msg is not None:
        #     return
        if self.spline is not None:
            return
        
        print("fit a spline to the reference path")
        self.ref_path_msg = ref_path_msg
        reference_path = ReferencePath()
        for pose in ref_path_msg.poses:
            reference_path.x.append(pose.pose.position.x)
            reference_path.y.append(pose.pose.position.y)
        
        self.spline = TorchSpline2D(reference_path)
    # def reference_path_callback(self, ref_path_msg):
    #     # print("path callback")
    #     self.ref_path_msg = ref_path_msg

    def bluetooth_callback(self, msg):
        if msg.axes[2] < -0.9 and self.enable_output == False:
            rospy.logwarn("Planning enabled (deadman switch pressed)")
        elif msg.axes[2] > -0.9 and self.enable_output:
            rospy.logwarn("Deadmanswitch enabled (deadman switch released)")
        
        self.enable_output = msg.axes[2] < -0.9



    def reset_env(self):
        # Call the service to reset the simulation
        self.reset_simulation_client()
        reset_msg = EmptyMsg()

        # Publish the Empty message to the specified topic
        self.reset_simulation_pub.publish(reset_msg)

    

    def guidance_service(self, x, y, psi, v):
        # Wait for the service to be available
        rospy.wait_for_service('/guidance_planner')
        rospy.logwarn("guidance service is called")

        # Call the service with the input state
        response = self.guidance_client(x, y, psi, v)
        print("number of guidance trajectories: ", len(response.trajectories))
        return response.trajectories


    def actuate(self, action):
        if self.enable_output:
            self.vel_cmd.linear.x  = action[0].item()
            self.vel_cmd.angular.z = action[1].item()
        else:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0

        # Publish the velocity command
        self.velocity_publisher.publish(self.vel_cmd)
        #self.rate.sleep()
    
    def publish_planned_trajectory(self, plan):
        poses = []

        for i, point in enumerate(plan):
            x, y = point[0].item(), point[1].item()  # Extract (x, y) from the tensor
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y 
            poses.append(pose)
        
        self.planned_trajectory.poses = poses
        self.plan_publisher.publish(self.planned_trajectory)
        '''
        for pose in poses:
            print("pose x: ", pose.pose.position.x, " pose y: ", pose.pose.position.y)
        '''
    
    '''
    def plot_mppi_rollouts(self, rollouts, costs):
        marker_array = MarkerArray()

        # Normalize costs to the range [0, 1] for color mapping
        min_cost = costs.min()
        max_cost = costs.max()
        normalized_costs = (costs - min_cost) / (max_cost - min_cost + 1e-6)  # Avoid division by zero

        # Convert normalized costs to a NumPy array (ensure it's on the CPU)
        normalized_costs = normalized_costs.cpu().numpy()

        for i in range(rollouts.shape[0]):  # Iterate over rollouts
            marker = Marker()
            marker.header.frame_id = "map"  # Use the appropriate frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rollouts"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set marker properties
            marker.scale.x = 0.05  # Line width
            # Map cost to color (e.g., green for low cost, red for high cost)
            color = plt.cm.jet(normalized_costs[i])  # Jet colormap gives vibrant colors
            marker.color.r = color[0]  # Red
            marker.color.g = color[1]  # Green
            marker.color.b = color[2]  # Blue
            marker.color.a = 0.8  # Transparency

            # Add points to the marker
            for t in range(rollouts.shape[1]):  # Iterate over horizon
                point = Point()
                point.x = rollouts[i, t, 0]  # x-coordinate
                point.y = rollouts[i, t, 1]  # y-coordinate
                point.z = 0.0  # z-coordinate
                marker.points.append(point)

            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.rollouts_publisher.publish(marker_array)
    '''

    def plot_mppi_rollouts(self, rollouts, costs):
        marker_array = MarkerArray()
        
        min_cost = torch.min(costs)
        max_cost = torch.max(costs)

        # Get indices of the 10 lowest costs
        best_costs, topk_indices = torch.topk(costs, k=30, largest=False)
        best_rollouts = rollouts[topk_indices]


        # Ensure costs are on the CPU and converted to NumPy
        best_costs = best_costs.cpu().numpy()
        # Normalize cost to a range [0,1] for interpolation
        min_cost = min(best_costs)
        max_cost = max(best_costs)
        cost_range = max_cost - min_cost if max_cost > min_cost else 1

        for i in range(best_rollouts.shape[0]):  # Iterate over rollouts
            marker = Marker()
            marker.header.frame_id = "map"  # Use the appropriate frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rollouts"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set marker properties
            marker.scale.x = 0.05  # Line width

            norm_cost = (best_costs[i] - min_cost) / cost_range  # Normalize to [0,1]
    
            marker.color.r = norm_cost  # More red as cost increases
            marker.color.g = 1.0 - norm_cost  # Less green as cost increases
            marker.color.b = 0.0  # No blue
            marker.color.a = 0.8  # Transparency


            '''
            # Assign color based on cost
            if costs[i] > 100:
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0  # No green
                marker.color.b = 0.0  # No blue
            else:
                marker.color.r = 0.0  # No red
                marker.color.g = 1.0  # Green
                marker.color.b = 0.0  # No blue
            marker.color.a = 0.8  # Transparency
            '''

            # Add points to the marker
            for t in range(best_rollouts.shape[1]):  # Iterate over horizon
                point = Point()
                point.x = best_rollouts[i, t, 0]  # x-coordinate
                point.y = best_rollouts[i, t, 1]  # y-coordinate
                point.z = 0.0  # z-coordinate
                marker.points.append(point)

            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.rollouts_publisher.publish(marker_array)




