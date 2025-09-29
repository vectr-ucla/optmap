"""
Code taken from https://github.com/cattaneod/PointNetVlad-Pytorch/blob/master/models/PointNetVlad.py with trivial
modifications.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from math import sqrt


class NetVLADLoupe(nn.Module):
    """
    The NetVLAD neural network, details check the paper NetVLAD and paper PointNetVLAD.

    Args:
        feature_size: (int) The size of the feature layers (number of local descriptors).
        max_samples: (int) The length of a single descriptor (width).
        cluster_size: (int) The size of the clusters.
        output_dim: (int) The dimension of the output vector.
        gating: (bool) Whether to use gatingContext.
        add_batch_norm: (bool) Whether to add batch normalization
    """
    def __init__(self, feature_size, max_samples, cluster_size, output_dim,
                 gating=True, add_batch_norm=True, is_training=True):
        super().__init__()
        self.feature_size = feature_size
        self.max_samples = max_samples
        self.cluster_size = cluster_size
        self.output_dim = output_dim
        self.gating = gating
        self.add_batch_norm = add_batch_norm
        self.is_training = is_training
        self.softmax = nn.Softmax(dim=-1)

        # add weights variables
        self.cluster_weights = nn.Parameter(1 / sqrt(feature_size) *
                                            torch.randn(feature_size, cluster_size))                   # parameters w
        self.cluster_weights2 = nn.Parameter(1 / sqrt(feature_size) *
                                             torch.randn(1, feature_size, cluster_size))               # parameters c_k
        self.hidden_weights = nn.Parameter(1 / sqrt(feature_size) *
                                           torch.randn(feature_size * cluster_size, output_dim))       # output weights

        # determine cluster norm and biases
        if self.add_batch_norm:
            self.bn1 = nn.BatchNorm1d(cluster_size)
            self.cluster_biases = None
        else:
            self.bn1 = None
            self.cluster_biases = nn.Parameter(1 / sqrt(feature_size) * torch.randn(cluster_size))

        self.bn2 = nn.BatchNorm1d(output_dim)

        # apply GatingContext filter before output the descriptor
        if self.gating:
            self.context_gating = GatingContext(output_dim, add_batch_norm=add_batch_norm)

    def forward(self, x):
        """
        feedforward pass of the NetVLAD neural network

        Args:
            x: (torch.tensor) with dimension (batch_size, feature_size, max_samples, 1).
                Considering the output of the last layer of the previous nn as H * W * D, which can be considered as
                D dimensional descriptors extracted at H * W spatial locations.
                Here H = 1, W = max_samples, D = feature_size.
        """
        x = x.transpose(1, 3).contiguous()
        x = x.view(-1, self.max_samples, self.feature_size)

        # soft-assignment branch
        activation = x @ self.cluster_weights
        if self.add_batch_norm:
            activation = activation.view(-1, self.cluster_size)
            activation = self.bn1(activation)
            activation = activation.view(-1, self.max_samples, self.cluster_size)
        else:
            activation += self.cluster_biases
        activation = self.softmax(activation)
        activation = activation.view(-1, self.max_samples, self.cluster_size)   # seems redundant here

        a_sum = activation.sum(-2, keepdim=True)
        a = a_sum * self.cluster_weights2

        # VLAD core
        x = x.view(-1, self.max_samples, self.feature_size)     # seems redundant here
        activation = activation.transpose(2, 1)
        vlad = activation @ x
        vlad = vlad.transpose(2, 1)
        vlad -= a

        # reform the output as a 1D vector
        vlad = F.normalize(vlad, p=2, dim=1)
        vlad = vlad.reshape(-1, self.feature_size * self.cluster_size)
        vlad = F.normalize(vlad, p=2, dim=1)
        vlad = vlad @ self.hidden_weights
        # vlad = self.bn2(vlad)     # overlaptransformer skip this line

        # apply gatingContext filter
        if self.gating:
            vlad = self.context_gating(vlad)

        vlad = F.normalize(vlad, p=2, dim=1)
        return vlad


class GatingContext(nn.Module):
    """
    A filter applies before the output of NetVlad descriptor.
    """

    def __init__(self, dim, add_batch_norm=True):
        """
        Define a small filter to determine the weight of each element in NetVLAD descriptor.

        Args:
            dim: (int) the dimension of the descriptor output by NetVLAD.
            add_batch_norm: (bool) whether to use batch normalization (if not add biases).
        """
        super().__init__()
        self.dim = dim
        self.add_batch_norm = add_batch_norm
        self.gating_weights = nn.Parameter(1 / sqrt(dim) * torch.randn(dim, dim))
        self.sigmoid = nn.Sigmoid()

        if self.add_batch_norm:
            self.gating_biases = None
            self.bn1 = nn.BatchNorm1d(dim)
        else:
            self.gating_biases = nn.Parameter(1 / sqrt(dim) * torch.randn(dim))
            self.bn1 = None

    def forward(self, x):
        gates = x @ self.gating_weights

        if self.add_batch_norm:
            gates = self.bn1(gates)
        else:
            gates = gates + self.gating_biases

        gates = self.sigmoid(gates)
        activation = x * gates

        return activation


if __name__ == '__main__':
    torch.manual_seed(0)
    net_vlad = NetVLADLoupe(feature_size=1024, max_samples=900, cluster_size=64,
                            output_dim=256, gating=True, add_batch_norm=True,
                            is_training=True)
    # input  (bs, 1024, 900, 1)
    input1 = F.normalize(torch.randn((30, 1024, 900, 1)), dim=1)
    input2 = input1.clone()
    input2 = input2[:, :, torch.randperm(input2.shape[2]), :]
    diff = torch.norm(input2 - input1)

    print("==================================")
    print(f'Norm of the difference before NetVLAD: {diff:.3f}.')

    with torch.no_grad():
        net_vlad.eval()
        output1 = net_vlad(input1)
        output2 = net_vlad(input2)
        diff = torch.norm(output2 - output1)
        print(f'Norm of the difference after NetVLAD: {diff:.3f}.')
        print(torch.norm(output1))