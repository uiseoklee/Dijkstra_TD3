"""
Grid based Dijkstra planning

author: Atsushi Sakai(@Atsushi_twi)
"""

"""
DijkstraNode & ImageGazeboTransformation

author: UiSeok Lee
"""


import os
import matplotlib.pyplot as plt
import math
import numpy as np
import cv2
from sklearn.linear_model import LinearRegression

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry


show_animation = True


class DijkstraNode(Node):
    def __init__(self, image_path, grid_size, robot_radius):
        super().__init__('dijkstra')

        """
        Initialise variables
        """

        self.image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if self.image is None:
            raise ValueError("Image not found or the path is incorrect")

        grid_size = grid_size
        robot_radius = robot_radius

        self.sx = 0.0
        self.sy = 0.0

        # set obstacle positions
        self.ox, self.oy = [], []

        # Decide how much contrast to consider as an obstacle
        threshold = 127
        _, obstacles = cv2.threshold(self.image, threshold, 255, cv2.THRESH_BINARY_INV)

        # Extract the coordinates of obstacles.
        obstacle_coords = np.column_stack(np.where(obstacles > 0))

        # Set obstacle location
        # In the image coordinate system, the y coordinate comes first, but in the Gazebo coordinate system, the x coordinate comes first.
        # Therefore, change the columns and rows in obstacle_coords to create ox and oy lists.
        self.ox = obstacle_coords[:, 1].tolist()  # x-axis coordinate
        self.oy = obstacle_coords[:, 0].tolist()  # y-axis coordinate

        self.rx = None
        self.ry = None
        
        self.dijkstra = Dijkstra(self.ox, self.oy, grid_size, robot_radius)
        self.converter = ImageGazeboTransformation()

        self.current_position = None
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.planning_active = False

        """
        Initialise ROS publishers, subscribers and clients
        """
        
        qos = QoSProfile(depth=10)

        # Initialize publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', qos)
        
        # Initialize subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

    """
    Callback functions and relevant functions
    """
    def odometry_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.sx, self.sy = self.converter.gazebo_to_image_coordinates(
            self.current_position.x, self.current_position.y)
        if self.rx is None and self.ry is None:
            self.update_plot()
    
    def update_plot(self):
        if self.current_position:
            self.ax.clear()
            self.ax.imshow(self.image, cmap='gray')
            self.ax.plot(self.sx, self.sy, "og")
            self.ax.scatter(self.ox, self.oy, s=0.1, color='k')
            plt.draw()

    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            self.planning_active = True
            # Execute planning on click
            gx, gy = event.xdata, event.ydata
            self.execute_planning(gx, gy)

    def execute_planning(self, goal_x, goal_y):
        self.rx, self.ry = self.dijkstra.planning(self.sx, self.sy, goal_x, goal_y)

        current_path_index = 0

        while current_path_index < len(self.rx) and self.planning_active:
            # To subscribe to the current location(self.sx, self.sy)
            rclpy.spin_once(self)

            target_x, target_y = self.rx[current_path_index], self.ry[current_path_index]

            gazebo_x, gazebo_y = self.converter.image_to_gazebo_coordinates(target_x, target_y)

            dx = self.current_position.x - gazebo_x
            dy = self.current_position.y - gazebo_y
            distance = math.sqrt(dx**2 + dy**2)

            # If within threshold, move to next target location
            threshold = 0.4
            if distance <= threshold:
                current_path_index += 1
                if current_path_index >= len(self.rx):
                    print("Reached the goal")
                    self.planning_active = False
                    break

            # Issue target location
            pose = Pose()
            pose.position.x, pose.position.y = gazebo_x, gazebo_y
            self.goal_pose_pub.publish(pose)

            plt.pause(0.001)

            # Path visualization
            self.ax.clear()
            self.ax.imshow(self.image, cmap='gray')
            self.ax.scatter(self.ox, self.oy, s=0.1, color='k')
            self.ax.plot(self.sx, self.sy, "go")
            self.ax.plot(self.rx, self.ry, "-r")
            plt.draw()


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while True:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        ## Invert the route and change the order from starting point to destination point (by UiSeok)
        rx.reverse()
        ry.reverse()

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class ImageGazeboTransformation:
    def __init__(self):
        """
        Different coordinates must be provided for each map.
        Select 4 random coordinates from gazebo and match the selected coordinates to the image.
        That is, 4 gazebo coordinates and 4 image coordinates are recorded.
        """
        
        # Gazebo coordinates (x, y)
        self.gazebo_coords = np.array([
            [6.20 , -0.16],
            [2.32, 4.63],
            [1.11, -0.19],
            [-6.31, 0.94]
        ])

        # Image coordinates (x, y)
        self.image_coords = np.array([
            [203.26, 73.71],
            [106.70, 151.83],
            [203.26, 176.78],
            [181.56, 324.34]
        ])

        # Rotate image coordinates by 90 degrees to align with Gazebo's orientation
        # Rotation matrix for 90 degrees: [cos(90), -sin(90); sin(90), cos(90)] = [0, -1; 1, 0]
        rotated_image_coords = np.dot(self.image_coords, np.array([[0, -1], [1, 0]]))

        # Linear Regression Model
        self.model = LinearRegression()
        self.model.fit(rotated_image_coords, self.gazebo_coords)

        self.inverse_model = LinearRegression()
        self.inverse_model.fit(self.gazebo_coords, self.image_coords)

    def image_to_gazebo_coordinates(self, image_x, image_y):
        """
        Converts image coordinates to gazebo coordinates using the previously trained linear regression model.

        Parameters:
        image_x (float): The x-coordinate in the image reference frame.
        image_y (float): The y-coordinate in the image reference frame.

        Returns:
        tuple: A tuple (gazebo_x, gazebo_y) representing the converted coordinates in the gazebo reference frame.
        """

        # Rotate the image coordinate by 90 degrees to align with Gazebo's orientation
        rotated_image_coord = np.dot(np.array([[image_x, image_y]]), np.array([[0, -1], [1, 0]]))

        # Use the model to predict the Gazebo coordinate
        predicted_gazebo_coord = self.model.predict(rotated_image_coord)

        return predicted_gazebo_coord[0]
    
    def gazebo_to_image_coordinates(self, gazebo_x, gazebo_y):
        # Convert Gazebo coordinates to image coordinates
        predicted_image_coord = self.inverse_model.predict(np.array([[gazebo_x, gazebo_y]]))
        
        return predicted_image_coord[0][0], predicted_image_coord[0][1]

def main():
    rclpy.init()
    print(__file__ + " start!!")

    # File directory of the occupancy grid map to run the dijkstra algorithm.
    image_path = os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_drl/turtlebot3_drl/drl_dijkstra/map.png'

    # Dijkstra setting
    grid_size = 4.9  # [m]
    robot_radius = 4.9  # [m]
    
    dijkstra_node = DijkstraNode(image_path, grid_size, robot_radius)
    
    while rclpy.ok():
        rclpy.spin_once(dijkstra_node)

        plt.pause(0.001)

if __name__ == '__main__':
    main()