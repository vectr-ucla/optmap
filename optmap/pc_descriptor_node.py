#!/usr/bin/env python3
import os
import sys
import torch
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from custom_interfaces.msg import Descriptor

from optmap.model.projection import RangeProjection
from optmap.model.overlap_transformer import OverlapTransformer32

from ament_index_python.packages import get_package_share_directory

class PointCloudDescriptor(Node):
    def __init__(self, weights):

        self.seq = 0

        # init and load model
        self.weights = weights
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        checkpoint = torch.load(self.weights, weights_only=False)
        self.model = OverlapTransformer32().to(self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()

        # init node      
        super().__init__('pc_descriptor_node')
        self.logger = self.get_logger()
        
        self.pc_sub = self.create_subscription(PointCloud2, 'pointcloud', self.callback, 1)
        self.descriptor_pub = self.create_publisher(Descriptor, 'descriptor', 1000)

        # load range image projector
        # Need to rebuild (colcon build) to update parameters
        self.declare_parameter('fov_up', 22.5)
        self.declare_parameter('fov_down', -22.5)
        self.fov_up = self.get_parameter('fov_up').get_parameter_value().double_value
        self.fov_down = self.get_parameter('fov_down').get_parameter_value().double_value
        self.projector = RangeProjection(fov_up=self.fov_up, fov_down=self.fov_down)

        self.logger.info('NN NODE INITIALIZED.')

    def callback(self, msg):
        # read pointcloud from ros msg
        point_cloud = point_cloud2.read_points(
            msg,
            skip_nans=True,
            field_names=["x", "y", "z"],
            reshape_organized_cloud=True
        )

        points = np.hstack([
            point_cloud['x'].reshape(-1, 1),
            point_cloud['y'].reshape(-1, 1),
            point_cloud['z'].reshape(-1, 1)
        ])

        # compute descriptor
        img = self.projector.doProjection(points)
        img_tensor = torch.tensor(img).unsqueeze(0).unsqueeze(0).float().to(self.device)
        descriptor = self.model(img_tensor)

        # publish data
        data = descriptor.cpu().detach().numpy().squeeze().tolist()
        descriptor_msg = Descriptor()
        descriptor_msg.header = msg.header
        descriptor_msg.id = self.seq
        descriptor_msg.data = data

        # publish descriptor
        self.descriptor_pub.publish(descriptor_msg)

        self.seq = self.seq + 1

def main(args=None):

    package_path = get_package_share_directory('optmap')
    workspace_path = os.path.abspath(os.path.join(package_path, '../../../..'))
    weights_path = os.path.join(workspace_path, 'src/optmap/optmap/model/best.pth.tar')

    rclpy.init(args=args)

    generator = PointCloudDescriptor(weights_path)
    rclpy.spin(generator)

    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()