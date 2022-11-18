# Punyo Soft-Bubble Sensor - Copyright 2022 Toyota Research Institute. All rights reserved.

import os

import cv2
import numpy as np
import torch
import torchvision.transforms as T
from ament_index_python import get_package_share_directory
from torchvision.models.optical_flow import raft_large

import punyo.utils.flow_vis as flow_vis

RAFT_SMALL_WEIGHTS_DEFAULT = "raft_small_C_T_V2-01064c6d.pth"
RAFT_LARGE_WEIGHTS_DEFAULT = "raft_large_C_T_SKHT_V2-ff5fadd5.pth"


class RaftEstimator:
    """ RAFT: Recurrent All Pairs Field Transforms for Optical Flow
        https://github.com/princeton-vl/RAFT"""

    def __init__(self, device="cuda", debug=False):
        """ Initialize the RAFT estimator. Device options: cuda or cpu """
        self._device = device
        self._debug = debug

        # Load weights
        package_share_directory = get_package_share_directory('punyo')
        weights_path = os.path.join(package_share_directory, 'models', RAFT_LARGE_WEIGHTS_DEFAULT)
        pretrained_weights = torch.load(weights_path, map_location=torch.device(self._device))

        # Setup model
        self._model = raft_large()  # raft_small()
        self._model.load_state_dict(pretrained_weights)
        self._model.to(self._device)
        self._model.eval()

    def load_image(self, img):
        img = torch.from_numpy(img).permute(2, 0, 1).float()  # (H, W, C) -> (C, H, W)
        return img[None].to(self._device)

    def calculate_raft_flow(self, origin_frame, current_frame):
        """ Returns the flow (the last iteration out of 12 iterations of the model) """
        img1 = self.load_image(origin_frame)
        img2 = self.load_image(current_frame)

        img1 = 2 * (img1 / 255.0) - 1.0
        img2 = 2 * (img2 / 255.0) - 1.0

        list_of_flows = self._model(img1, img2)
        if self._debug:
            print(f"type = {type(list_of_flows)}")
            print(f"length = {len(list_of_flows)} = number of iterations of the model")

        predicted_flows = list_of_flows[-1]
        if self._debug:
            print(f"dtype = {predicted_flows.dtype}")
            print(f"shape = {predicted_flows.shape} = (N, 2, H, W)")
            print(f"min = {predicted_flows.min()}, max = {predicted_flows.max()}")

        return predicted_flows

    # @torch.no_grad()
    # def viz(self, img, flo):
    #     flo = flo[0].permute(1, 2, 0).cpu().numpy()
    #
    #     # map flow to rgb image
    #     flo = flow_vis.flow_to_image(flo)
    #     img_flo = np.concatenate([img, flo], axis=0)
    #
    #     cv2.imshow('image', img_flo[:, :, [2, 1, 0]] / 255.0)
    #     cv2.waitKey(1)

    # def calculate_raft_flow_with_batch(self, origin_frame, current_frame):
    #     origin_frame = torch.from_numpy(origin_frame).permute(2, 0, 1) # (H, W, C) -> (C, H, W)
    #     current_frame = torch.from_numpy(current_frame).permute(2, 0, 1) # (H, W, C) -> (C, H, W)
    #
    #     img1_batch = torch.stack([origin_frame])
    #     img2_batch = torch.stack([current_frame])
    #     #
    #     img1 = self.preprocess(img1_batch).to(self.device)
    #     img2 = self.preprocess(img2_batch).to(self.device)
    #
    #     # TODO
    #
    # def preprocess(self, batch):
    #     transforms = T.Compose(
    #         [
    #             T.ConvertImageDtype(torch.float32),
    #             T.Normalize(mean=0.5, std=0.5),  # map [0, 1] to [-1, 1]
    #             T.Resize(size=(520, 960)),
    #         ]
    #     )
    #     batch = transforms(batch)
    #     return batch
