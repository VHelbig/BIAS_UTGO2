#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points  # Import read_points
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal Node has been started.')

        # Create a subscription to the /utlidar/cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10  # QoS profile depth
        )
        self.spacing=0.01
        self.max_range=10
        self.frame_translate=np.array([0,0,0.35])
        self.frame_rotmat=R.from_quat(np.array([-0.991,0,0.130,0])).as_matrix().squeeze()
        print(self.frame_rotmat)
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg: PointCloud2):
        # Print relevant information about the PointCloud2 message
        # self.get_logger().info(f'Received PointCloud2 message:')
        # self.get_logger().info(f'  Width: {msg.width}')
        # self.get_logger().info(f'  Height: {msg.height}')
        # self.get_logger().info(f'  Is Dense: {msg.is_dense}')
        # self.get_logger().info(f'  Number of Points: {msg.width * msg.height}')

        # Extract and print x, y, z coordinates
        grid=np.zeros((int(self.max_range/self.spacing), int(self.max_range/self.spacing)), dtype=np.bool8)
        points=list()
        for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z= point
            point=np.array([x,y,z])
            point=self.frame_rotmat@point+self.frame_translate
            points.append(point)
            if point[2]>0.01 and point[2]<1:
                point_2d=point[:2]
                point_2d=(point_2d+self.max_range/2)/self.spacing
                if point_2d[0]>0 and point_2d[0]<grid.shape[0] and point_2d[1]>0 and point_2d[1]<grid.shape[1]:
                    grid[int(point_2d[0]), int(point_2d[1])]=True
        
        points=np.array(points)

        fig=plt.figure()
        ax=fig.add_subplot(projection='3d')
        ax.scatter(points[:,0], points[:,1], points[:,2], c='r', marker='o')
        # plot a square as the bottom
        ax.plot_surface(np.array([-self.max_range/2, self.max_range/2, self.max_range/2, -self.max_range/2]), 
                        np.array([-self.max_range/2, -self.max_range/2, self.max_range/2, self.max_range/2]), 
                        np.array([0,0,0,0]), alpha=0.5)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show()
        
        exit()
    

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()