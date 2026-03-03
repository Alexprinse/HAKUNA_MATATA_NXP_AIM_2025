# Copyright 2025 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

import math
import time
import numpy as np
import cv2
from typing import Optional, Tuple
import asyncio
import threading

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from synapse_msgs.msg import Status
from synapse_msgs.msg import WarehouseShelf

from scipy.ndimage import label, center_of_mass
from scipy.spatial.distance import euclidean
from sklearn.decomposition import PCA

import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
mpl.rcParams['toolbar'] = 'None'

QOS_PROFILE_DEFAULT = 10
SERVER_WAIT_TIMEOUT_SEC = 5.0

PROGRESS_TABLE_GUI = True


class WindowProgressTable:
	def __init__(self, root, shelf_count):
		self.root = root
		self.root.title("Shelf Objects & QR Link")
		self.root.attributes("-topmost", True)

		self.row_count = 2
		self.col_count = shelf_count

		self.boxes = []
		for row in range(self.row_count):
			row_boxes = []
			for col in range(self.col_count):
				box = tk.Text(root, width=10, height=3, wrap=tk.WORD, borderwidth=1,
						  relief="solid", font=("Helvetica", 14))
				box.insert(tk.END, "NULL")
				box.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
				row_boxes.append(box)
			self.boxes.append(row_boxes)

		# Make the grid layout responsive.
		for row in range(self.row_count):
			self.root.grid_rowconfigure(row, weight=1)
		for col in range(self.col_count):
			self.root.grid_columnconfigure(col, weight=1)

	def change_box_color(self, row, col, color):
		self.boxes[row][col].config(bg=color)

	def change_box_text(self, row, col, text):
		self.boxes[row][col].delete(1.0, tk.END)
		self.boxes[row][col].insert(tk.END, text)

box_app = None
def run_gui(shelf_count):
	global box_app
	root = tk.Tk()
	box_app = WindowProgressTable(root, shelf_count)
	root.mainloop()


class WarehouseExplore(Node):

	"""Initializes warehouse explorer node with the required publishers and subscriptions."""

	def __init__(self):
		# --- Directional Exploration State ---
		self.directional_move_count = 0
		self.directional_max_moves = 5  # Maximum times to move in direction before giving up
		self.directional_step_dist = 2.0  # meters per move
		self.frontier_max_dist = 15.0  # meters, only consider frontiers within this range
		# --- Initial Exploration State ---
		self.initial_exploration_done = False

		# Track exploration origin type: 'world' or 'shelf'
		self.exploration_origin_type = 'world'
		self.exploration_origin = (0.0, 0.0)

		# QR angle update flag - simpler than complex condition checking
		self.allow_qr_angle_update = False

		# Track shelves traversed for each side
		self.shelves_traversed_object = set()
		self.shelves_traversed_qr = set()
		
		# Store current shelf being traversed (for marking as visited after completion)
		self.current_traversed_shelf = None

		super().__init__('warehouse_explore')

		self.action_client = ActionClient(
			self,
			NavigateToPose,
			'/navigate_to_pose')

		self.subscription_pose = self.create_subscription(
			PoseWithCovarianceStamped,
			'/pose',
			self.pose_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_global_map = self.create_subscription(
			OccupancyGrid,
			'/global_costmap/costmap',
			self.global_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_simple_map = self.create_subscription(
			OccupancyGrid,
			'/map',
			self.simple_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_status = self.create_subscription(
			Status,
			'/cerebri/out/status',
			self.cerebri_status_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_behavior = self.create_subscription(
			BehaviorTreeLog,
			'/behavior_tree_log',
			self.behavior_tree_log_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_shelf_objects = self.create_subscription(
			WarehouseShelf,
			'/shelf_objects',
			self.shelf_objects_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Publisher for output image (for debug purposes).
		self.publisher_qr_decode = self.create_publisher(
			CompressedImage,
			"/debug_images/qr_code",
			QOS_PROFILE_DEFAULT)

		self.publisher_shelf_data = self.create_publisher(
			WarehouseShelf,
			"/shelf_data",
			QOS_PROFILE_DEFAULT)

		self.declare_parameter('shelf_count', 5) #2 , 4 , 3 , 5
		self.declare_parameter('initial_angle', 045.0) #135 , 040.6 , 045 , 045
		self.declare_parameter('enable_visualization', True)

		self.shelf_count = \
			self.get_parameter('shelf_count').get_parameter_value().integer_value
		self.initial_angle = \
			self.get_parameter('initial_angle').get_parameter_value().double_value
		self.enable_visualization = \
			self.get_parameter('enable_visualization').get_parameter_value().bool_value
		self.next_shelf_angle = self.initial_angle  # Angle in degrees

		# --- Robot State ---
		self.armed = False
		self.logger = self.get_logger()

		# --- Robot Pose ---
		self.pose_curr = PoseWithCovarianceStamped()
		self.buggy_pose_x = 0.0
		self.buggy_pose_y = 0.0
		self.buggy_center = (0.0, 0.0)
		self.world_center = (0.0, 0.0)

		# --- Map Data ---
		self.simple_map_curr = None
		self.global_map_curr = None

		# --- Goal Management ---
		self.xy_goal_tolerance = 0.5
		self.goal_completed = True  # No goal is currently in-progress.
		self.goal_handle_curr = None
		self.cancelling_goal = False
		self.recovery_threshold = 5

		# --- Goal Creation ---
		self._frame_id = "map"

		# --- Exploration Parameters ---
		self.max_step_dist_world_meters = 7.0
		self.min_step_dist_world_meters = 4.0
		self.full_map_explored_count = 0

		# --- Shelf Traversal ---
		self.shelf_goal_queue = []  # List of (goal_pose, goal_type, shelf_id)
		self.shelf_traversal_active = False
		self.current_shelf_id = None

		# --- QR Code Data ---
		self.qr_code_str = "Empty"
		if PROGRESS_TABLE_GUI:
			self.table_row_count = 0
			self.table_col_count = 0

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()

		# Initialize previous object count number
		self.previous_object_count_number = 0

		#pub objs
		self.pub_object_names = []
		self.pub_object_counts = []
		self.pub_object_count_number = 0
		self.prev_qr_code_str = ""
		self.prev_shelf_number = 0
		self.pub_done = False
		self.Angle_update = False



	def pose_callback(self, message):
		"""Callback function to handle pose updates.

		Args:
			message: ROS2 message containing the current pose of the rover.

		Returns:
			None
		"""
		self.pose_curr = message
		self.buggy_pose_x = message.pose.pose.position.x
		self.buggy_pose_y = message.pose.pose.position.y
		self.buggy_center = (self.buggy_pose_x, self.buggy_pose_y)

	def visualize_shelves_and_corridor(self, map_data, map_info):
		"""Visualize detected shelves and search corridor on the map."""
		try:
			# Create figure if it doesn't exist
			if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number):
				plt.ion()
				self.fig, self.ax = plt.subplots(figsize=(10, 8))
				self.fig.canvas.manager.set_window_title("Shelf Detection & Corridor Visualization")
				plt.subplots_adjust(left=0, right=1, top=0.95, bottom=0.05)
			
			# Clear previous plot
			self.ax.clear()
			self.ax.axis('off')
			
			# Get map dimensions
			height, width = map_info.height, map_info.width
			map_array = np.array(map_data).reshape((height, width))
			
			# Create RGB image for map
			image = np.zeros((height, width, 3), dtype=np.uint8)
			for y in range(height):
				for x in range(width):
					value = map_array[y, x]
					if value == 0:
						image[y, x] = [0, 0, 0]  # Black (free space)
					elif value == 100:
						image[y, x] = [128, 128, 128]  # Gray (obstacles)
					else:  # value == -1
						image[y, x] = [255, 255, 255]  # White (unknown)
			
			# Display the map
			self.ax.imshow(image, origin='lower')
			
			# Draw detected shelves
			if hasattr(self, 'shelf_clusters') and self.shelf_clusters:
				for i, shelf in enumerate(self.shelf_clusters):
					# Convert world coordinates to map coordinates
					shelf_x_map, shelf_y_map = self.get_map_coord_from_world_coord(
						shelf['center_x'], shelf['center_y'], map_info)
					
					# Draw shelf center as red circle
					circle = patches.Circle((shelf_x_map, shelf_y_map), radius=5, 
										  color='red', fill=True, alpha=0.8)
					self.ax.add_patch(circle)
					
					# Draw shelf orientation as blue line
					orientation = shelf['orientation']
					length_pixels = int(shelf['length'] / map_info.resolution)
					dx = (length_pixels // 2) * math.cos(orientation)
					dy = (length_pixels // 2) * math.sin(orientation)
					
					self.ax.plot([shelf_x_map - dx, shelf_x_map + dx], 
							   [shelf_y_map - dy, shelf_y_map + dy], 
							   'b-', linewidth=3, alpha=0.8)
					
					# Add shelf label
					self.ax.text(shelf_x_map + 10, shelf_y_map + 10, f'S{i+1}', 
							   color='red', fontsize=10, fontweight='bold')
			
			# Draw robot position
			robot_x_map, robot_y_map = self.get_map_coord_from_world_coord(
				self.buggy_center[0], self.buggy_center[1], map_info)
			robot_circle = patches.Circle((robot_x_map, robot_y_map), radius=8, 
										color='green', fill=True, alpha=0.9)
			self.ax.add_patch(robot_circle)
			self.ax.text(robot_x_map + 15, robot_y_map + 15, 'ROBOT', 
					   color='green', fontsize=12, fontweight='bold')
			
			# Draw search corridor
			if self.exploration_origin_type == 'world':
				origin = self.world_center
			else:
				origin = self.exploration_origin
			
			# Convert origin to map coordinates
			origin_x_map, origin_y_map = self.get_map_coord_from_world_coord(
				origin[0], origin[1], map_info)
			
			# Draw corridor parameters
			corridor_width = 3.0  # meters (from check_shelf_in_direction)
			corridor_length = 25.0  # meters
			
			# Convert to map coordinates
			width_pixels = int(corridor_width / map_info.resolution)
			length_pixels = int(corridor_length / map_info.resolution)
			
			# Calculate corridor rectangle
			angle_rad = math.radians(self.next_shelf_angle)
			cos_a = math.cos(angle_rad)
			sin_a = math.sin(angle_rad)
			
			# Corridor corners in map coordinates
			half_width = width_pixels // 2
			corners = [
				(origin_x_map - half_width * sin_a, origin_y_map + half_width * cos_a),
				(origin_x_map + half_width * sin_a, origin_y_map - half_width * cos_a),
				(origin_x_map + half_width * sin_a + length_pixels * cos_a, 
				 origin_y_map - half_width * cos_a + length_pixels * sin_a),
				(origin_x_map - half_width * sin_a + length_pixels * cos_a, 
				 origin_y_map + half_width * cos_a + length_pixels * sin_a)
			]
			
			# Draw corridor as transparent yellow rectangle
			corridor_patch = patches.Polygon(corners, closed=True, 
											color='yellow', alpha=0.3, 
											edgecolor='orange', linewidth=2)
			self.ax.add_patch(corridor_patch)
			
		# Initialize variables if not already done
			# Draw direction arrow
			arrow_end_x = origin_x_map + (length_pixels // 2) * cos_a
			arrow_end_y = origin_y_map + (length_pixels // 2) * sin_a
			self.ax.annotate('', xy=(arrow_end_x, arrow_end_y), 
							xytext=(origin_x_map, origin_y_map),
							arrowprops=dict(arrowstyle='->', color='orange', lw=3))
			
			# Add title and info
			title = f"Shelf Detection | Angle: {self.next_shelf_angle:.1f}° | Shelves: {len(self.shelf_clusters) if hasattr(self, 'shelf_clusters') else 0}"
			self.ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
			
			# Add legend
			legend_elements = [
				patches.Patch(color='red', label='Detected Shelves'),
				patches.Patch(color='green', label='Robot Position'),
				patches.Patch(color='yellow', alpha=0.3, label='Search Corridor'),
				patches.Patch(color='orange', label='Search Direction')
			]
			self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(0.98, 0.98))
			
			# Set axis limits and aspect
			self.ax.set_xlim(0, width)
			self.ax.set_ylim(0, height)
			self.ax.set_aspect('equal')
			
			# Update display
			plt.pause(0.01)
			plt.show()
			
		except Exception as e:
			self.logger.error(f"Visualization error: {e}")

	def simple_map_callback(self, message):
		"""Callback function to handle simple map updates and identify shelves (robust version)."""
		self.simple_map_curr = message
		map_info = self.simple_map_curr.info
		height, width = map_info.height, map_info.width
		map_array = np.array(self.simple_map_curr.data).reshape((height, width))

		# Find all cells with value 100 (shelf/obstacle)
		shelf_indices = np.argwhere(map_array == 100)
		if shelf_indices.size == 0:
			self.shelf_clusters = []
			if not hasattr(self, '_last_shelf_print') or self._last_shelf_print != 'none':
				self.logger.info("No obstacles found in map")
				self._last_shelf_print = 'none'
			# Still visualize the map and corridor even when no shelves are detected
			if self.enable_visualization:
				self.visualize_shelves_and_corridor(self.simple_map_curr.data, self.simple_map_curr.info)
			return

		# Convert map coords to world coords for DBSCAN
		obstacle_points_world = [self.get_world_coord_from_map_coord(x, y, map_info) for y, x in shelf_indices]
		obstacle_points_world = np.array(obstacle_points_world)

		# DBSCAN clustering in world coordinates (robust params)
		from sklearn.cluster import DBSCAN
		dbscan = DBSCAN(eps=1.5, min_samples=5)
		cluster_labels = dbscan.fit_predict(obstacle_points_world)

		# Process each cluster
		shelf_candidates = []
		n_clusters = len(set(cluster_labels)) - (1 if -1 in cluster_labels else 0)
		# self.logger.info(f"DBSCAN found {n_clusters} clusters")
		for cluster_id in range(n_clusters):
			cluster_mask = (cluster_labels == cluster_id)
			cluster_points = obstacle_points_world[cluster_mask]
			if len(cluster_points) < 10:
				continue
			center_x = np.mean(cluster_points[:, 0])
			center_y = np.mean(cluster_points[:, 1])
			if len(cluster_points) >= 2:
				centered_points = cluster_points - np.mean(cluster_points, axis=0)
				from sklearn.decomposition import PCA
				pca = PCA(n_components=2)
				pca.fit(centered_points)
				principal_axes = pca.components_
				principal_axis = principal_axes[0]
				orientation = np.arctan2(principal_axis[1], principal_axis[0])
				if orientation < 0:
					orientation += np.pi
				projected_points = pca.transform(centered_points)
				min_proj = np.min(projected_points, axis=0)
				max_proj = np.max(projected_points, axis=0)
				length_world = max_proj[0] - min_proj[0]
				width_world = max_proj[1] - min_proj[1]
				if width_world > length_world:
					length_world, width_world = width_world, length_world
					orientation = (orientation + np.pi/2) % np.pi
			else:
				orientation = 0.0
				length_world = 0.1
				width_world = 0.1
			shelf_candidate = {
				'center_x': center_x,
				'center_y': center_y,
				'orientation': orientation,
				'length': length_world,
				'width': width_world,
				'cluster_size': len(cluster_points),
				'cluster_points': cluster_points,
				'shelf_id': self.generate_shelf_id(center_x, center_y)  # Use coordinate-based ID
			}
			shelf_candidates.append(shelf_candidate)

		# Merge and filter shelves
		merged_shelves = self.merge_nearby_clusters(shelf_candidates)
		filtered_shelves = self.filter_shelves_by_dimensions(merged_shelves)

		# Store detected shelves
		self.shelf_clusters = filtered_shelves
		# Only print shelf info if changed
		shelf_state_str = str([(round(s['center_x'],2), round(s['center_y'],2), round(s['orientation'],2), round(s['length'],2), round(s['width'],2)) for s in filtered_shelves])
		if not hasattr(self, '_last_shelf_print') or self._last_shelf_print != shelf_state_str:
			if filtered_shelves:
				# self.logger.info(f"Detected {len(filtered_shelves)} shelf(s):")
				for i, shelf in enumerate(filtered_shelves):
					cx, cy = shelf['center_x'], shelf['center_y']
					theta = shelf['orientation']
					shelf_id = shelf.get('shelf_id', self.generate_shelf_id(cx, cy))
					# self.logger.info(f"  Shelf {i+1}: ID={shelf_id}, center=({cx:.2f}, {cy:.2f}), orientation={theta:.2f} rad, dims=({shelf['length']:.2f}, {shelf['width']:.2f})")
			else:
				self.logger.info("No shelves detected after filtering")
			self._last_shelf_print = shelf_state_str
		
		# Visualize shelves and corridor (if enabled)
		# if self.enable_visualization:
		# 	self.visualize_shelves_and_corridor(self.simple_map_curr.data, self.simple_map_curr.info)

	def merge_nearby_clusters(self, shelf_candidates):
		from scipy.spatial.distance import euclidean
		merged_shelves = []
		used_indices = set()
		for i, shelf1 in enumerate(shelf_candidates):
			if i in used_indices:
				continue
			merged_shelf = shelf1.copy()
			cluster_group = [shelf1]
			used_indices.add(i)
			for j, shelf2 in enumerate(shelf_candidates):
				if j in used_indices or i == j:
					continue
				distance = euclidean((shelf1['center_x'], shelf1['center_y']), (shelf2['center_x'], shelf2['center_y']))
				merge_threshold = 1.0
				if distance < merge_threshold:
					cluster_group.append(shelf2)
					used_indices.add(j)
			if len(cluster_group) > 1:
				total_size = sum(shelf['cluster_size'] for shelf in cluster_group)
				weighted_x = sum(shelf['center_x'] * shelf['cluster_size'] for shelf in cluster_group) / total_size
				weighted_y = sum(shelf['center_y'] * shelf['cluster_size'] for shelf in cluster_group) / total_size
				largest_cluster = max(cluster_group, key=lambda x: x['cluster_size'])
				all_points = []
				for shelf in cluster_group:
					if 'cluster_points' in shelf:
						all_points.extend(shelf['cluster_points'].tolist())
				merged_shelf = {
					'center_x': weighted_x,
					'center_y': weighted_y,
					'orientation': largest_cluster['orientation'],
					'length': max(shelf['length'] for shelf in cluster_group),
					'width': max(shelf['width'] for shelf in cluster_group),
					'cluster_size': total_size,
					'cluster_points': np.array(all_points) if all_points else largest_cluster.get('cluster_points', np.array([])),
					'shelf_id': self.generate_shelf_id(weighted_x, weighted_y)  # Use coordinate-based ID
				}
			merged_shelves.append(merged_shelf)
		return merged_shelves

	def filter_shelves_by_dimensions(self, shelf_candidates):
		filtered_shelves = []
		# Set expected shelf dimensions and tolerance (adjust as needed)
		expected_length = 1.35
		expected_width = 0.55
		tolerance_length = 0.25
		tolerance_width = 0.15
		for shelf in shelf_candidates:
			length = shelf['length']
			width = shelf['width']
			dims = sorted([length, width])
			if (abs(dims[0] - expected_width) < tolerance_width and abs(dims[1] - expected_length) < tolerance_length):
				filtered_shelves.append(shelf)
		return filtered_shelves

	def global_map_callback(self, message):
		"""Callback function to handle global map updates (directional exploration logic with fallback to frontier exploration)."""
		self.global_map_curr = message

		# If shelf traversal is active, do not continue exploration
		if self.shelf_traversal_active:
			return

		# Only proceed if no goal is in progress
		if not self.goal_completed:
			return

		# Track if we've already tried moving in the direction for this angle
		if not hasattr(self, '_tried_directional_move'):
			self._tried_directional_move = False
		if not hasattr(self, 'directional_move_count'):
			self.directional_move_count = 0

		# Determine exploration origin
		if self.exploration_origin_type == 'world':
			origin = self.world_center
		else:
			origin = self.exploration_origin

		# If we have shelves detected
		if hasattr(self, 'shelf_clusters') and self.shelf_clusters:
			# Filter out shelves that have already been traversed (object or QR)
			unvisited_shelves = []
			for shelf in self.shelf_clusters:
				cx, cy = shelf['center_x'], shelf['center_y']
				shelf_id = shelf.get('shelf_id', self.generate_shelf_id(cx, cy))
				if shelf_id in self.shelves_traversed_object and shelf_id in self.shelves_traversed_qr:
					continue  # Skip shelves already traversed for both sides
				unvisited_shelves.append(shelf)
			
			# Check for nearby shelves in 3m radius (but skip on first step-by-step move)
			if hasattr(self, 'directional_move_count') and self.directional_move_count > 1 and self._tried_directional_move:
				robot_x, robot_y = self.buggy_center[0], self.buggy_center[1]
				nearby_shelf = self.check_shelves_in_radius(unvisited_shelves, robot_x, robot_y, radius=3.0)
				if nearby_shelf is not None and self.is_shelf_far_enough_from_visited(nearby_shelf, min_distance=3.0):
					self.logger.info(f"Found unvisited shelf within 3m radius during step-by-step movement! Switching to nearby shelf.")
					shelf = nearby_shelf
				else:
					shelf = self.check_shelf_in_direction(unvisited_shelves, self.next_shelf_angle, origin=origin)
			else:
				shelf = self.check_shelf_in_direction(unvisited_shelves, self.next_shelf_angle, origin=origin)
			if shelf is not None:
				# Check if this shelf is far enough from previously visited shelves
				if self.is_shelf_far_enough_from_visited(shelf, min_distance=3.0):
					self.logger.info(f"Shelf found in direction {self.next_shelf_angle} deg and is far enough from visited shelves, traversing.")
					# Note: Will add to visited list only after traversal completion
					self.shelf_goal_queue.clear()
					cx, cy = shelf['center_x'], shelf['center_y']
					orientation = shelf['orientation']
					shelf_id = shelf.get('shelf_id', self.generate_shelf_id(cx, cy))  # Use coordinate-based ID
					
					# 🛡️ SAFETY GOAL - Position between robot and shelf center for natural approach
					robot_x, robot_y = self.buggy_center[0], self.buggy_center[1]  # Current robot position
					
					# Calculate vector from robot to shelf center
					dx_to_shelf = cx - robot_x
					dy_to_shelf = cy - robot_y
					distance_to_shelf = math.hypot(dx_to_shelf, dy_to_shelf)
					
					# Position safety goal at 60% of the way from robot to shelf (closer to shelf)
					safety_ratio = 0.6  # 60% of the distance
					safety_x = robot_x + safety_ratio * dx_to_shelf
					safety_y = robot_y + safety_ratio * dy_to_shelf
					
					# Orient robot to face toward shelf center
					safety_theta = math.atan2(cy - safety_y, cx - safety_x)
					
					safety_goal_pose = self.create_goal_from_world_coord(safety_x, safety_y, safety_theta)
					approach_distance = distance_to_shelf * (1 - safety_ratio)  # Remaining distance to shelf
					self.logger.info(f"Queueing SAFETY approach goal: pos=({safety_x:.2f},{safety_y:.2f}), yaw={safety_theta:.2f}, {approach_distance:.1f}m from shelf")
					self.shelf_goal_queue.append((safety_goal_pose, 'safety', shelf_id))
					
					# Traverse object side only if not done for this shelf
					if shelf_id not in self.shelves_traversed_object:
						# Create two object viewing positions at different distances
						obj_distances = [2.0, 2.3, 2.7, 3.1, 2.3]   # closer and farther positions
						for i, d_obj in enumerate(obj_distances):
							car_x_obj = cx + d_obj * math.cos(orientation + math.pi/2)
							car_y_obj = cy + d_obj * math.sin(orientation + math.pi/2)
							obj_theta = math.atan2(cy - car_y_obj, cx - car_x_obj)
							goal_pose_obj = self.create_goal_from_world_coord(car_x_obj, car_y_obj, obj_theta)
							position_name = "close" if i == 0 else "far"
							self.logger.info(f"Queueing OBJECT view goal ({position_name}) for shelf: pos=({car_x_obj:.2f},{car_y_obj:.2f}), yaw={obj_theta:.2f}, dist={d_obj}m")
							self.shelf_goal_queue.append((goal_pose_obj, 'object', shelf_id))
						self.shelves_traversed_object.add(shelf_id)
					
					# Traverse QR side only if not done for this shelf
					if shelf_id not in self.shelves_traversed_qr:
						# Create two QR viewing positions at different distances
						qr_distances = [2.2, 2.4]  # closer and farther positions
						for i, d_qr in enumerate(qr_distances):
							car_x_qr = cx + d_qr * math.cos(orientation)
							car_y_qr = cy + d_qr * math.sin(orientation)
							qr_theta = math.atan2(cy - car_y_qr, cx - car_x_qr)
							goal_pose_qr = self.create_goal_from_world_coord(car_x_qr, car_y_qr, qr_theta)
							position_name = "close" if i == 0 else "far"
							self.logger.info(f"Queueing QR view goal ({position_name}) for shelf: pos=({car_x_qr:.2f},{car_y_qr:.2f}), yaw={qr_theta:.2f}, dist={d_qr}m")
							self.shelf_goal_queue.append((goal_pose_qr, 'qr', shelf_id))
						self.shelves_traversed_qr.add(shelf_id)
					
					if self.shelf_goal_queue:
						self.shelf_traversal_active = True
						self.current_shelf_id = None
						# 🎯 Store shelf reference for marking as visited after completion
						self.current_traversed_shelf = shelf
						self._tried_directional_move = False  # Reset for next cycle
						# Set shelf center as new exploration origin for next phase
						self.exploration_origin_type = 'shelf'
						self.exploration_origin = (cx, cy)
						self.directional_move_count = 0  # Reset move count after shelf found
						self.send_next_shelf_goal()
					else:
						self.logger.info(f"Shelf {shelf_id} already traversed for both sides.")
					return
				else:
					self.logger.info(f"Shelf found in direction {self.next_shelf_angle} deg but too close to previously visited shelf (< 3m). Proceeding to frontier exploration.")
			else:
				# Check if shelf exists but is too close to visited shelves
				all_shelves_in_direction = self.check_shelf_in_direction(self.shelf_clusters, self.next_shelf_angle, origin=origin)
				if all_shelves_in_direction is not None and not self.is_shelf_far_enough_from_visited(all_shelves_in_direction, min_distance=0.5):
					self.logger.info(f"Shelf found in direction {self.next_shelf_angle} deg but too close to previously visited shelf (< 3m). Proceeding to frontier exploration.")
				else:
					self.logger.info(f"No suitable shelf found in direction {self.next_shelf_angle} deg. Proceeding to frontier exploration.")
			
			# No suitable shelf found, proceed directly to frontier exploration
			height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
			map_array = np.array(self.global_map_curr.data).reshape((height, width))
			frontiers = self.get_frontiers_for_space_exploration(map_array)
			map_info = self.global_map_curr.info
			# Filter frontiers by distance
			desired_angle_rad = math.radians(self.next_shelf_angle)
			dir_x = math.cos(desired_angle_rad)
			dir_y = math.sin(desired_angle_rad)
			angle_threshold = math.pi / 4  # 60 degrees tolerance
			filtered_frontiers = []
			for fy, fx in frontiers:
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, map_info)
				dx = fx_world - self.buggy_center[0]
				dy = fy_world - self.buggy_center[1]
				norm = math.hypot(dx, dy)
				if norm == 0 or norm > self.frontier_max_dist:
					continue
				frontier_dir_x = dx / norm
				frontier_dir_y = dy / norm
				dot = frontier_dir_x * dir_x + frontier_dir_y * dir_y
				dot = max(min(dot, 1.0), -1.0)
				angle_diff = math.acos(dot)
				if angle_diff <= angle_threshold:
					filtered_frontiers.append((fy, fx, norm))
			if filtered_frontiers:
				# Pick a frontier in the middle (average distance) aligned with the desired direction
				distances = [item[2] for item in filtered_frontiers]
				min_dist = min(distances)
				max_dist = max(distances)
				avg_dist = (min_dist + max_dist) / 2.0
				best_frontier = min(filtered_frontiers, key=lambda item: abs(item[2] - avg_dist))
				fy, fx, chosen_dist = best_frontier
				goal = self.create_goal_from_map_coord(fx, fy, map_info)
				self.send_goal_from_world_pose(goal)
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, map_info)
				self.logger.info(f"Sent middle-distance directional frontier goal to ({fx}, {fy}) at distance {chosen_dist:.2f}m aligned with {self.next_shelf_angle} deg")
				self._tried_directional_move = False
				return
			else:
				# No frontiers found aligned with the desired direction, move step by step
				if self.directional_move_count < self.directional_max_moves:
					ox, oy = origin
					dist = self.directional_step_dist * (self.directional_move_count + 1)
					angle_rad = math.radians(self.next_shelf_angle)
					goal_x = ox + dist * math.cos(angle_rad)
					goal_y = oy + dist * math.sin(angle_rad)
					goal_pose = self.create_goal_from_world_coord(goal_x, goal_y, angle_rad)
					self.send_goal_from_world_pose(goal_pose)
					self.logger.info(f"No aligned frontiers found. Moving step by step to ({goal_x:.2f}, {goal_y:.2f}) in direction {self.next_shelf_angle} deg. Move count: {self.directional_move_count + 1}")
					self.directional_move_count += 1
					self._tried_directional_move = True
					# Mark initial exploration as done after first goal is sent
					if not self.initial_exploration_done:
						self._waiting_for_initial_goal = True
					return
				else:
					self.logger.info("Maximum directional moves reached. Giving up on this direction.")
					self.directional_move_count = 0
					self._tried_directional_move = False
					# If shelves lost, reset origin to world
					self.exploration_origin_type = 'world'
					self.exploration_origin = self.world_center
					return
		else:
			# If no shelves detected, start exploring to find them
			if self.directional_move_count == 0:
				# Start exploring in the initial direction
				ox, oy = self.world_center
				dist = self.directional_step_dist * (self.directional_move_count + 1)
				angle_rad = math.radians(self.next_shelf_angle)
				goal_x = ox + dist * math.cos(angle_rad)
				goal_y = oy + dist * math.sin(angle_rad)
				goal_pose = self.create_goal_from_world_coord(goal_x, goal_y, angle_rad)
				self.send_goal_from_world_pose(goal_pose)
				self.logger.info(f"No shelves detected. Starting exploration by moving to ({goal_x:.2f}, {goal_y:.2f}) in direction {self.next_shelf_angle} deg. Move count: {self.directional_move_count + 1}")
				self.directional_move_count += 1
				self._tried_directional_move = True
				if not self.initial_exploration_done:
					self._waiting_for_initial_goal = True
				return
			else:
				# Continue exploring by looking for frontiers or moving step by step
				height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
				map_array = np.array(self.global_map_curr.data).reshape((height, width))
				frontiers = self.get_frontiers_for_space_exploration(map_array)
				map_info = self.global_map_curr.info
				# Filter frontiers by distance
				desired_angle_rad = math.radians(self.next_shelf_angle)
				dir_x = math.cos(desired_angle_rad)
				dir_y = math.sin(desired_angle_rad)
				angle_threshold = math.pi / 4  # 45 degrees tolerance
				filtered_frontiers = []
				for fy, fx in frontiers:
					fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, map_info)
					dx = fx_world - self.buggy_center[0]
					dy = fy_world - self.buggy_center[1]
					norm = math.hypot(dx, dy)
					if norm == 0 or norm > self.frontier_max_dist:
						continue
					frontier_dir_x = dx / norm
					frontier_dir_y = dy / norm
					dot = frontier_dir_x * dir_x + frontier_dir_y * dir_y
					dot = max(min(dot, 1.0), -1.0)
					angle_diff = math.acos(dot)
					if angle_diff <= angle_threshold:
						filtered_frontiers.append((fy, fx, norm))
				if filtered_frontiers:
					# Pick a frontier in the middle (average distance) aligned with the desired direction
					distances = [item[2] for item in filtered_frontiers]
					min_dist = min(distances)
					max_dist = max(distances)
					avg_dist = (min_dist + max_dist) / 2.0
					best_frontier = min(filtered_frontiers, key=lambda item: abs(item[2] - avg_dist))
					fy, fx, chosen_dist = best_frontier
					goal = self.create_goal_from_map_coord(fx, fy, map_info)
					self.send_goal_from_world_pose(goal)
					fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, map_info)
					self.logger.info(f"No shelves detected. Exploring frontier at ({fx}, {fy}) at distance {chosen_dist:.2f}m aligned with {self.next_shelf_angle} deg")
					self._tried_directional_move = False
					return
				else:
					# No frontiers found aligned with the desired direction, move step by step
					if self.directional_move_count < self.directional_max_moves:
						ox, oy = self.world_center
						dist = self.directional_step_dist * (self.directional_move_count + 1)
						angle_rad = math.radians(self.next_shelf_angle)
						goal_x = ox + dist * math.cos(angle_rad)
						goal_y = oy + dist * math.sin(angle_rad)
						goal_pose = self.create_goal_from_world_coord(goal_x, goal_y, angle_rad)
						self.send_goal_from_world_pose(goal_pose)
						self.logger.info(f"No shelves or aligned frontiers found. Moving step by step to ({goal_x:.2f}, {goal_y:.2f}) in direction {self.next_shelf_angle} deg. Move count: {self.directional_move_count + 1}")
						self.directional_move_count += 1
						self._tried_directional_move = True
						if not self.initial_exploration_done:
							self._waiting_for_initial_goal = True
						return
					else:
						self.logger.info("Maximum directional moves reached without finding shelves. Resetting exploration.")
						self.directional_move_count = 0
						self._tried_directional_move = False
						return

	def generate_shelf_id(self, center_x, center_y):
		"""
		Generate a unique, persistent shelf ID based on rounded coordinates.
		This ensures the same shelf gets the same ID across robot runs.
		"""
		return (round(center_x, 1), round(center_y, 1))

	def check_shelf_in_direction(self, shelves, angle_deg, origin=None, corridor_width=3.0, corridor_length=25.0):
		"""
		Checks if a shelf is present within a rectangular region (corridor)
		in the specified direction (angle in degrees) from the buggy center.
		Returns the closest shelf if multiple are found.
		Returns:
			dict or None: Closest shelf dictionary if found, else None.
		"""
		theta_rad = math.radians(angle_deg)
		if origin is None:
			x0, y0 = self.buggy_center
		else:
			x0, y0 = origin
		dir_x = math.cos(theta_rad)
		dir_y = math.sin(theta_rad)
		half_width = corridor_width / 2
		
		closest_shelf = None
		closest_distance = float('inf')
		
		for idx, shelf in enumerate(shelves):
			if 'center_x' not in shelf or 'center_y' not in shelf:
				continue
			sx = shelf['center_x']
			sy = shelf['center_y']
			dx = sx - x0
			dy = sy - y0
			proj_len = dx * dir_x + dy * dir_y
			perp_len = abs(-dir_y * dx + dir_x * dy)
			
			if 0 <= proj_len <= corridor_length and perp_len <= half_width:
				distance = math.sqrt(dx*dx + dy*dy)
				if distance < closest_distance:
					closest_distance = distance
					closest_shelf = shelf
		
		return closest_shelf

	def check_shelves_in_radius(self, shelves, center_x, center_y, radius=3.0):
		"""
		Check for shelves within a given radius from a center point.
		Returns the closest shelf if any found within radius.
		"""
		closest_shelf = None
		closest_distance = float('inf')
		
		for shelf in shelves:
			if 'center_x' not in shelf or 'center_y' not in shelf:
				continue
			
			sx = shelf['center_x']
			sy = shelf['center_y']
			distance = math.sqrt((sx - center_x)**2 + (sy - center_y)**2)
			
			if distance <= radius and distance < closest_distance:
				closest_distance = distance
				closest_shelf = shelf
		
		return closest_shelf

	def update_qr_angle_if_valid(self, qr_str):
		"""Update QR angle only if we're currently at a QR viewing goal."""
		
		if self.allow_qr_angle_update:
			angle = self.extract_angle_from_qr(qr_str)
			if angle is not None:
				old_angle = self.next_shelf_angle
				self.next_shelf_angle = angle
				self._tried_directional_move = False
				self.Angle_update = True
				# Reset flag after successful angle update
				self.allow_qr_angle_update = False
				return True
			else:
				return False
		else:
			current_goal_type = getattr(self, 'last_goal_type', 'unknown')
			return False

	def extract_angle_from_qr(self, qr_str):
		"""Extract angle in degrees from QR string with improved parsing."""
		try:
			if qr_str and len(qr_str) >= 6:
				# Example: '1_315.0_Ad5PqIlXvPqApGFdLXEboS'
				# Split by underscore to get parts
				parts = qr_str.split('_')
				if len(parts) >= 2:
					# Second part should be the angle
					angle_str = parts[1]
					angle = float(angle_str)
					return angle
		except (ValueError, IndexError) as e:
			pass
		return None

	def extract_shelf_number_from_qr(self, qr_str):
		"""Extract shelf number from QR string."""
		try:
			if qr_str and len(qr_str) >= 1:
				# Example: '1_315.0_Ad5PqIlXvPqApGFdLXEboS'
				# Shelf number = int(qr_str[0])
				shelf_number = int(qr_str[0])
				return shelf_number
		except (ValueError, IndexError) as e:
			pass
		return None

	async def robust_qr_scan(self, scan_time=5.0, interval=0.5):
		"""Scan for QR code for a period, retrying every interval seconds."""
		from pyzbar import pyzbar
		start_time = time.time()
		detected_qr = None
		while time.time() - start_time < scan_time:
			await asyncio.sleep(interval)
			if hasattr(self, 'latest_camera_image'):
				image = self.latest_camera_image
				decoded_objects = pyzbar.decode(image)
				if decoded_objects:
					detected_qr = decoded_objects[0].data.decode('utf-8')
					# Update QR string and try to update angle
					# if detected_qr != self.qr_code_str:
					# 	self.qr_code_str = detected_qr
					# 	# Try to update angle using the safe method
					# 	# update_result = self.update_qr_angle_if_valid(detected_qr)
					break
		return detected_qr

	def send_next_shelf_goal(self):
		"""Send the next goal from the shelf goal queue."""
		if not self.shelf_goal_queue:
			self.logger.info("Shelf traversal complete. No more shelf goals.")
			
			# 🎯 Mark shelf as visited ONLY after successful traversal completion
			if hasattr(self, 'current_traversed_shelf') and self.current_traversed_shelf:
				self.add_shelf_to_visited(self.current_traversed_shelf)
				self.logger.info(f"✅ Shelf traversal completed - marked shelf as visited!")
				self.current_traversed_shelf = None  # Clear reference
			
			self.shelf_traversal_active = False
			# Reset QR angle update flag when shelf traversal ends
			self.allow_qr_angle_update = False
			return
		goal_pose, goal_type, shelf_id = self.shelf_goal_queue.pop(0)
		self.current_shelf_id = shelf_id
		self.last_goal_type = goal_type  # Track last goal type for waiting logic
		
		# Handle different goal types for QR angle updates and logging
		if goal_type == 'qr':
			self.allow_qr_angle_update = True
		elif goal_type == 'safety':
			self.allow_qr_angle_update = False
			self.logger.info(f"🛡️ Moving to SAFETY staging position for shelf {shelf_id}")
		else:
			self.allow_qr_angle_update = False
			
		self.logger.info(f"Sending {goal_type} goal for shelf {shelf_id}")
		self.send_goal_from_world_pose(goal_pose)

	def get_frontiers_for_space_exploration(self, map_array):
		"""Identifies frontiers for space exploration.

		Args:
			map_array: 2D numpy array representing the map.

		Returns:
			frontiers: List of tuples representing frontier coordinates.
		"""
		frontiers = []
		inflation_threshold = 70  # Cells with cost >= this are considered risky (tune as needed)
		for y in range(1, map_array.shape[0] - 1):
			for x in range(1, map_array.shape[1] - 1):
				if map_array[y, x] == -1:  # Unknown space and not visited.
					neighbors_complete = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
						(y - 1, x - 1),
						(y + 1, x - 1),
						(y - 1, x + 1),
						(y + 1, x + 1)
					]

					near_obstacle = False
					for ny, nx in neighbors_complete:
						if map_array[ny, nx] > inflation_threshold:  # Obstacles or inflation region
							near_obstacle = True
							break
					if near_obstacle:
						continue

					neighbors_cardinal = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
					]

					for ny, nx in neighbors_cardinal:
						if map_array[ny, nx] == 0:  # Free space.
							frontiers.append((ny, nx))
							break

		return frontiers



	def publish_debug_image(self, publisher, image):
		"""Publishes images for debugging purposes.

		Args:
			publisher: ROS2 publisher of the type sensor_msgs.msg.CompressedImage.
			image: Image given by an n-dimensional numpy array.

		Returns:
			None
		"""
		if image.size:
			message = CompressedImage()
			_, encoded_data = cv2.imencode('.jpg', image)
			message.format = "jpeg"
			message.data = encoded_data.tobytes()
			publisher.publish(message)

	def camera_image_callback(self, message):
		"""Callback function to handle incoming camera images and decode QR codes.

		Args:
			message: ROS2 message of the type sensor_msgs.msg.CompressedImage.

		Returns:
			None
		"""
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		# Store latest image for robust QR scan
		self.latest_camera_image = image

		# --- QR Decoding Logic ---
		try:
			from pyzbar import pyzbar
			decoded_objects = pyzbar.decode(image)
			if decoded_objects:
				# Take the first detected QR code
				qr_data = decoded_objects[0].data.decode('utf-8')
				if qr_data != self.qr_code_str and self.allow_qr_angle_update:
					self.qr_code_str = qr_data
					# Use the safe QR angle update method
					self.update_qr_angle_if_valid(qr_data)
					
					# # Always check if shelf number matches shelf count (completion check)
					# shelf_number = 0
					# shelf_number = self.extract_shelf_number_from_qr(qr_data)
					# if shelf_number is not None and shelf_number == self.shelf_count:
					# 	self.logger.info(f"All shelves completed! QR shelf number ({shelf_number}) matches shelf count ({self.shelf_count}). Navigating to origin and stopping.")
					# 	# Navigate to origin (0,0) and then shutdown
					# 	goal_pose = self.create_goal_from_world_coord(0.0, 0.0, 0.0)
					# 	self.send_goal_from_world_pose(goal_pose)
					# 	return

			else:
				# Optionally clear or keep last QR if nothing found
				pass
		except Exception as e:
			pass

		# Optional line for visualizing image on foxglove.
		self.publish_debug_image(self.publisher_qr_decode, image)

	def cerebri_status_callback(self, message):
		"""Callback function to handle cerebri status updates.

		Args:
			message: ROS2 message containing cerebri status.

		Returns:
			None
		"""
		if message.mode == 3 and message.arming == 2:
			self.armed = True
		else:
			# Initialize and arm the CMD_VEL mode.
			msg = Joy()
			msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
			msg.axes = [0.0, 0.0, 0.0, 0.0]
			self.publisher_joy.publish(msg)

	def behavior_tree_log_callback(self, message):
		"""Alternative method for checking goal status.

		Args:
			message: ROS2 message containing behavior tree log.

		Returns:
			None
		"""
		for event in message.event_log:
			if (event.node_name == "FollowPath" and
				event.previous_status == "SUCCESS" and
				event.current_status == "IDLE"):
				# self.goal_completed = True
				# self.goal_handle_curr = None
				pass

	def shelf_objects_callback(self, message):
		"""Callback function to handle shelf objects updates."""
		
		# Initialize variables if not already done
		allowed_objects = {"horse", "clock", "zebra", "potted plant", "cup", "banana", "car", "teddy bear"}

		# Filter invalid objects
		filtered_names = []
		filtered_counts = []
		removed_objects = []
		shelf_data_message = WarehouseShelf()

		for name, count in zip(message.object_name, message.object_count):
			if name in allowed_objects:
				filtered_names.append(name)
				filtered_counts.append(count)
			else:
				removed_objects.append((name, count))

		self.shelf_objects_curr = message
		self.curr_object_names = filtered_names
		self.curr_object_counts = filtered_counts
		self.curr_object_count_number = sum(self.curr_object_counts)

		if  (filtered_names != self.pub_object_names and self.curr_object_count_number >= self.pub_object_count_number and self.curr_object_count_number <= 6) :
			self.pub_object_names = filtered_names
			self.pub_object_counts = filtered_counts
			self.pub_object_count_number = sum(self.pub_object_counts)
			

		shelf_data_message.object_name = self.pub_object_names
		shelf_data_message.object_count = self.pub_object_counts
		shelf_data_message.qr_decoded = self.qr_code_str
		# print(f"shelf_data_message: {shelf_data_message}")

		if self.qr_code_str != 'Empty' and self.prev_qr_code_str != self.qr_code_str and self.Angle_update:
			self.publisher_shelf_data.publish(shelf_data_message)
			self.prev_qr_code_str = self.qr_code_str
			if PROGRESS_TABLE_GUI and self.qr_code_str:
				try:
					shelf_idx = int(self.qr_code_str[0]) - 1
					if 0 <= shelf_idx < self.shelf_count:
						obj_str = ""
						for name, count in zip(self.pub_object_names, self.pub_object_counts):
							obj_str += f"{name}: {count}\n"

						box_app.change_box_text(0, shelf_idx, obj_str)
						box_app.change_box_color(0, shelf_idx, "cyan")
						box_app.change_box_text(1, shelf_idx, self.qr_code_str)
						box_app.change_box_color(1, shelf_idx, "yellow")
						self.pub_done = True
						self.Angle_update = False
					else:
						self.get_logger().warn(f"⚠️ Shelf index {shelf_idx} out of bounds for GUI")
				except Exception as e:
					self.get_logger().error(f"❌ GUI update failed: {e}")
		if self.pub_done == True:
			self.pub_object_names = []
			self.pub_object_counts = []
			self.pub_object_count_number = 0
			self.pub_done = False
			
		


		# if self.curr_object_count_number > self.previous_object_count_number :
		# 	# Publish updated shelf data

		# 	shelf_data_message.object_name = self.curr_object_names
		# 	shelf_data_message.object_count = self.curr_object_counts
		# 	self.previous_object_count_number = self.curr_object_count_number
		# 	print(f"shelf_data_message: {shelf_data_message}")


		# if self.qr_code_str != 'Empty':
		# 	self.qr_code_str = message.qr_decoded if hasattr(message, 'qr_decoded') else 'Empty'
		# 	self.publisher_shelf_data.publish(shelf_data_message)
		# print(f"shelf_data_message_1: {shelf_data_message}")




		"""
		* Example for sending WarehouseShelf messages for evaluation.
			shelf_data_message = WarehouseShelf()

			shelf_data_message.object_name = ["car", "clock"]
			shelf_data_message.object_count = [1, 2]
			shelf_data_message.qr_decoded = "test qr string"

			self.publisher_shelf_data.publish(shelf_data_message)

		* Alternatively, you may store the QR for current shelf as self.qr_code_str.
			Then, add it as self.shelf_objects_curr.qr_decoded = self.qr_code_str
			Then, publish as self.publisher_shelf_data.publish(self.shelf_objects_curr)
			This, will publish the current detected objects with the last QR decoded.
		"""

		# Optional code for populating TABLE GUI with detected objects and QR data.
		"""
		if PROGRESS_TABLE_GUI:
			shelf = self.shelf_objects_curr
			obj_str = ""
			for name, count in zip(shelf.object_name, shelf.object_count):
				obj_str += f"{name}: {count}\n"

			box_app.change_box_text(self.table_row_count, self.table_col_count, obj_str)
			box_app.change_box_color(self.table_row_count, self.table_col_count, "cyan")
			self.table_row_count += 1

			box_app.change_box_text(self.table_row_count, self.table_col_count, self.qr_code_str)
			box_app.change_box_color(self.table_row_count, self.table_col_count, "yellow")
			self.table_row_count = 0
			self.table_col_count += 1
		"""

		# If QR decoded, update next_shelf_angle and reset directional move flag
		if hasattr(message, 'qr_decoded') and message.qr_decoded:
			# Use the safe QR angle update method
			# self.update_qr_angle_if_valid(message.qr_decoded)

			# Always check if shelf number matches shelf count (completion check)
			shelf_number = self.extract_shelf_number_from_qr(message.qr_decoded)
			if shelf_number is not None and shelf_number == self.shelf_count:
				self.logger.info(f"All shelves completed! QR shelf number ({shelf_number}) matches shelf count ({self.shelf_count}). Navigating to origin and stopping.")
				# Navigate to origin (0,0) and then shutdown
				goal_pose = self.create_goal_from_world_coord(0.0, 0.0, 0.0)
				self.send_goal_from_world_pose(goal_pose)
				# Set a timer to shutdown after reaching origin (give it time to reach)
				threading.Timer(10.0, lambda: rclpy.shutdown()).start()

	def rover_move_manual_mode(self, speed, turn):
		"""Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: The speed of the car in float. Range = [-1.0, +1.0];
				   Direction: forward for positive, reverse for negative.
			turn: Steer value of the car in float. Range = [-1.0, +1.0];
				  Direction: left turn for positive, right turn for negative.

		Returns:
			None
		"""
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, speed, 0.0, turn]
		self.publisher_joy.publish(msg)



	def cancel_goal_callback(self, future):
		"""
		Callback function executed after a cancellation request is processed.

		Args:
			future (rclpy.Future): The future is the result of the cancellation request.
		"""
		cancel_result = future.result()
		if cancel_result:
			self.logger.info("Goal cancellation successful.")
			self.cancelling_goal = False  # Mark cancellation as completed (success).
			return True
		else:
			self.logger.error("Goal cancellation failed.")
			self.cancelling_goal = False  # Mark cancellation as completed (failed).
			return False

	def cancel_current_goal(self):
		"""Requests cancellation of the currently active navigation goal."""
		if self.goal_handle_curr is not None and not self.cancelling_goal:
			self.cancelling_goal = True  # Mark cancellation in-progress.
			self.logger.info("Requesting cancellation of current goal...")
			cancel_future = self.action_client._cancel_goal_async(self.goal_handle_curr)
			cancel_future.add_done_callback(self.cancel_goal_callback)

	def goal_result_callback(self, future):
		"""
		Callback function executed when the navigation goal reaches a final result.

		Args:
			future (rclpy.Future): The future that is result of the navigation action.
		"""
		status = future.result().status
		# NOTE: Refer https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html.

		if status == GoalStatus.STATUS_SUCCEEDED:
			self.logger.info("Goal completed successfully!")
		else:
			self.logger.warn(f"Goal failed with status: {status}")

		self.goal_completed = True  # Mark goal as completed.
		self.goal_handle_curr = None  # Clear goal.

		# If initial exploration not done, mark as done after first goal result
		if not self.initial_exploration_done:
			self.initial_exploration_done = True
			self.logger.info("Initial exploration complete. Shelf traversal will be enabled on next map update.")
		# If shelf traversal is active, send next shelf goal
		if self.shelf_traversal_active:
			# Wait for 3 seconds if last goal was object-side
			if hasattr(self, 'last_goal_type') and self.last_goal_type == 'object':
				self.logger.info("Waiting 0.5 seconds after reaching object pose...")
				threading.Timer(0.1, self.send_next_shelf_goal).start()
			else:
				self.send_next_shelf_goal()

	def goal_response_callback(self, future):
		"""
		Callback function executed after the goal is sent to the action server.

		Args:
			future (rclpy.Future): The future that is server's response to goal request.
		"""
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.logger.warn('Goal rejected :(')
			self.goal_completed = True  # Mark goal as completed (rejected).
			self.goal_handle_curr = None  # Clear goal handle.
		else:
			self.logger.info('Goal accepted :)')
			self.goal_completed = False  # Mark goal as in progress.
			self.goal_handle_curr = goal_handle  # Store goal handle.

			get_result_future = goal_handle.get_result_async()
			get_result_future.add_done_callback(self.goal_result_callback)

	def goal_feedback_callback(self, msg):
		"""
		Callback function to receive feedback from the navigation action.

		Args:
			msg (nav2_msgs.action.NavigateToPose.Feedback): The feedback message.
		"""
		distance_remaining = msg.feedback.distance_remaining
		number_of_recoveries = msg.feedback.number_of_recoveries
		navigation_time = msg.feedback.navigation_time.sec
		estimated_time_remaining = msg.feedback.estimated_time_remaining.sec

		self.logger.debug(f"Recoveries: {number_of_recoveries}, "
				  f"Navigation time: {navigation_time}s, "
				  f"Distance remaining: {distance_remaining:.2f}, "
				  f"Estimated time remaining: {estimated_time_remaining}s")

		if number_of_recoveries > self.recovery_threshold and not self.cancelling_goal:
			self.logger.warn(f"Cancelling. Recoveries = {number_of_recoveries}.")
			self.cancel_current_goal()  # Unblock by discarding the current goal.

	def send_goal_from_world_pose(self, goal_pose):
		"""
		Sends a navigation goal to the Nav2 action server.

		Args:
			goal_pose (geometry_msgs.msg.PoseStamped): The goal pose in the world frame.

		Returns:
			bool: True if the goal was successfully sent, False otherwise.
		"""
		if not self.goal_completed or self.goal_handle_curr is not None:
			return False

		self.goal_completed = False  # Starting a new goal.

		goal = NavigateToPose.Goal()
		goal.pose = goal_pose

		if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
			self.logger.error('NavigateToPose action server not available!')
			return False

		# Send goal asynchronously (non-blocking).
		goal_future = self.action_client.send_goal_async(goal, self.goal_feedback_callback)
		goal_future.add_done_callback(self.goal_response_callback)

		return True



	def _get_map_conversion_info(self, map_info) -> Optional[Tuple[float, float]]:
		"""Helper function to get map origin and resolution."""
		if map_info:
			origin = map_info.origin
			resolution = map_info.resolution
			return resolution, origin.position.x, origin.position.y
		else:
			return None

	def get_world_coord_from_map_coord(self, map_x: int, map_y: int, map_info) \
					   -> Tuple[float, float]:
		"""Converts map coordinates to world coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			world_x = (map_x + 0.5) * resolution + origin_x
			world_y = (map_y + 0.5) * resolution + origin_y
			return (world_x, world_y)
		else:
			return (0.0, 0.0)

	def get_map_coord_from_world_coord(self, world_x: float, world_y: float, map_info) \
					   -> Tuple[int, int]:
		"""Converts world coordinates to map coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			map_x = int((world_x - origin_x) / resolution)
			map_y = int((world_y - origin_y) / resolution)
			return (map_x, map_y)
		else:
			return (0, 0)

	def _create_quaternion_from_yaw(self, yaw: float) -> Quaternion:
		"""Helper function to create a Quaternion from a yaw angle."""
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		q = Quaternion()
		q.x = 0.0
		q.y = 0.0
		q.z = sy
		q.w = cy
		return q

	def create_yaw_from_vector(self, dest_x: float, dest_y: float,
				   source_x: float, source_y: float) -> float:
		"""Calculates the yaw angle from a source to a destination point.
			NOTE: This function is independent of the type of map used.

			Input: World coordinates for destination and source.
			Output: Angle (in radians) with respect to x-axis.
		"""
		delta_x = dest_x - source_x
		delta_y = dest_y - source_y
		yaw = math.atan2(delta_y, delta_x)

		return yaw

	def create_goal_from_world_coord(self, world_x: float, world_y: float,
					 yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from world coordinates.
			NOTE: This function is independent of the type of map used.
		"""
		goal_pose = PoseStamped()
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		goal_pose.header.frame_id = self._frame_id

		goal_pose.pose.position.x = world_x
		goal_pose.pose.position.y = world_y

		if yaw is None and self.pose_curr is not None:
			# Calculate yaw from current position to goal position.
			source_x = self.pose_curr.pose.pose.position.x
			source_y = self.pose_curr.pose.pose.position.y
			yaw = self.create_yaw_from_vector(world_x, world_y, source_x, source_y)
		elif yaw is None:
			yaw = 0.0
		else:  # No processing needed; yaw is supplied by the user.
			pass

		goal_pose.pose.orientation = self._create_quaternion_from_yaw(yaw)

		pose = goal_pose.pose.position
		print(f"Goal created: ({pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f})")
		return goal_pose

	def create_goal_from_map_coord(self, map_x: int, map_y: int, map_info,
					   yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from map coordinates."""
		world_x, world_y = self.get_world_coord_from_map_coord(map_x, map_y, map_info)

		return self.create_goal_from_world_coord(world_x, world_y, yaw)

	def is_shelf_far_enough_from_visited(self, shelf, min_distance=1.0):
		"""
		Check if a shelf is at least min_distance away from any previously visited shelf.
		
		Args:
			shelf: Shelf dictionary with center_x and center_y
			min_distance: Minimum distance in meters (default 3.0)
		
		Returns:
			bool: True if shelf is far enough from all visited shelves, False otherwise
		"""
		if not hasattr(self, 'visited_shelf_centers'):
			self.visited_shelf_centers = []
		
		shelf_center = (shelf['center_x'], shelf['center_y'])
		
		for visited_center in self.visited_shelf_centers:
			distance = euclidean(shelf_center, visited_center)
			if distance < min_distance:
				return False
		
		return True
	
	def add_shelf_to_visited(self, shelf):
		"""Add a shelf center to the list of visited shelf centers."""
		if not hasattr(self, 'visited_shelf_centers'):
			self.visited_shelf_centers = []
		
		shelf_center = (shelf['center_x'], shelf['center_y'])
		self.visited_shelf_centers.append(shelf_center)
		self.logger.info(f"Added shelf center ({shelf_center[0]:.2f}, {shelf_center[1]:.2f}) to visited list")


def main(args=None):
	rclpy.init(args=args)

	warehouse_explore = WarehouseExplore()

	if PROGRESS_TABLE_GUI:
		gui_thread = threading.Thread(target=run_gui, args=(warehouse_explore.shelf_count,))
		gui_thread.start()

	rclpy.spin(warehouse_explore)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()