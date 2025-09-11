#!/usr/bin/env python3
import os
import sys
import torch
import numpy as np

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from optmap.msg import Descriptor

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)

from model.projection import RangeProjection
from model.overlap_transformer import OverlapTransformer32


class PointCloudDescriptor:
    def __init__(self, weights):
        # init and load model
        self.weights = weights
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        checkpoint = torch.load(self.weights, weights_only=False)
        self.model = OverlapTransformer32().to(self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()

        # init node
        rospy.init_node('nn_node', anonymous=True)      

        # load range image projector
        fov_up = rospy.get_param('~nn/fov_up')
        fov_down = rospy.get_param('~nn/fov_down')
        self.projector = RangeProjection(fov_up=fov_up, fov_down=fov_down)

        # subscribe pointcloud
        rospy.Subscriber('~pointcloud', PointCloud2, self.callback, queue_size=1, tcp_nodelay=True)
        self.descriptor_pub = rospy.Publisher('~descriptor', Descriptor, latch=True, queue_size=1000)
        rospy.loginfo('NN NODE INITIALIZED.')

    def callback(self, msg):
        # read pointcloud from ros msg
        pointcloud = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pointcloud = np.array(list(pointcloud))

        # compute descriptor
        img = self.projector.doProjection(pointcloud)
        img_tensor = torch.tensor(img).unsqueeze(0).unsqueeze(0).float().to(self.device)
        descriptor = self.model(img_tensor)

        # publish data
        data = descriptor.cpu().detach().numpy().squeeze().tolist()
        descriptor_msg = Descriptor()
        descriptor_msg.header = msg.header
        descriptor_msg.id = msg.header.seq
        descriptor_msg.data = data

        # publish descriptor
        self.descriptor_pub.publish(descriptor_msg)
        # rospy.loginfo(f'Published output descriptor.')


if __name__ == '__main__':
    weights_path = os.path.join(file_dir, 'best.pth.tar')

    try:
        generator = PointCloudDescriptor(weights_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('NN descriptor node terminated.')