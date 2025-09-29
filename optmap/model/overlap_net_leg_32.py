"""
Code taken from https://github.com/haomo-ai/OverlapTransformer/tree/master/modules with trivial modifications.
    - drop the NetVLAD layer in the original code.
    - use batch_first-True in transformer.
    - the original code in condition transformer=False has an output in shape (1, 512, width, 1) which does not match
      the shape requirement in paper (1, 1024, width, 1). Thus, add an addition layer to double the feature layer size.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
"""
    Feature extracter of OverlapTransformer.
    Args:
        height: the height of the range image (64 for KITTI sequences). 
                 This is an interface for other types LIDAR.
        width: the width of the range image (900, alone the lines of OverlapNet).
                This is an interface for other types LIDAR.
        channels: 1 for depth only in our work. 
                This is an interface for multiple cues.
        norm_layer: None in our work for better model.
        use_transformer: Whether to use MHSA.
"""


class OverlapNetLeg32(nn.Module):
    def __init__(self, height=32, width=512, channels=1, norm_layer=None, use_transformer=True):
        super(OverlapNetLeg32, self).__init__()
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d   # number of channels

        self.use_transformer = use_transformer

        self.conv1 = nn.Conv2d(channels, 16, kernel_size=(2, 1), stride=(2, 1), bias=False)
        self.bn1 = norm_layer(16)
        self.conv1_add = nn.Conv2d(16, 16, kernel_size=(5, 1), stride=(1, 1), bias=False)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=(3, 1), stride=(1, 1), bias=False)
        self.bn2 = norm_layer(32)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=(3, 1), stride=(1, 1), bias=False)
        self.bn3 = norm_layer(64)
        self.conv4 = nn.Conv2d(64, 64, kernel_size=(3, 1), stride=(1, 1), bias=False)
        self.bn4 = norm_layer(64)
        self.conv5 = nn.Conv2d(64, 128, kernel_size=(3, 1), stride=(1, 1), bias=False)
        self.bn5 = norm_layer(128)
        self.conv6 = nn.Conv2d(128, 128, kernel_size=(3, 1), stride=(1, 1), bias=False)
        self.bn6 = norm_layer(128)
        self.conv7 = nn.Conv2d(128, 128, kernel_size=(1, 1), stride=(2, 1), bias=False)
        self.bn7 = norm_layer(128)
        self.relu = nn.ReLU(inplace=True)

        encoder_layer = nn.TransformerEncoderLayer(d_model=256, nhead=4, dim_feedforward=1024, activation='relu',
                                                   batch_first=True, dropout=0.)
        self.transformer_encoder = torch.nn.TransformerEncoder(encoder_layer, num_layers=1)
        self.convLast1 = nn.Conv2d(128, 256, kernel_size=(1, 1), stride=(1, 1), bias=False)
        self.bnLast1 = norm_layer(256)
        self.convLast2 = nn.Conv2d(512, 1024, kernel_size=(1, 1), stride=(1, 1), bias=False)
        self.bnLast2 = norm_layer(1024)

        self.linear = nn.Linear(128 * width, 256)

        self.sigmoid = nn.Sigmoid()
        self.softmax = nn.Softmax()


    def forward(self, x_l):
        out_l = self.relu(self.conv1(x_l))
        out_l = self.relu(self.conv1_add(out_l))
        out_l = self.relu(self.conv2(out_l))
        out_l = self.relu(self.conv3(out_l))
        out_l = self.relu(self.conv4(out_l))
        out_l = self.relu(self.conv5(out_l))
        out_l = self.relu(self.conv6(out_l))
        out_l = self.relu(self.conv7(out_l))

        out_l_1 = out_l.permute(0, 1, 3, 2)             # (bs, 128, W, 1) [B, C, W, H]
        out_l_1 = self.relu(self.convLast1(out_l_1))    # (bs, 256, W, 1)

        if self.use_transformer:
            out_l = out_l_1.squeeze(3)                  # (bs, 256, W)
            out_l = out_l.permute(0, 2, 1)              # (bs, W, 256)
            out_l = self.transformer_encoder(out_l)     # True: (batch, seq, feature)
            out_l = out_l.permute(0, 2, 1)              # (bs, 256, W)
            out_l = out_l.unsqueeze(3)                  # (bs, 256, W, 1)
            out_l = torch.cat((out_l_1, out_l), dim=1)  # (bs, 512, W, 1)
            out_l = self.relu(self.convLast2(out_l))    # (bs, 1024, W, 1)
            out_l = F.normalize(out_l, dim=1)

        return out_l
