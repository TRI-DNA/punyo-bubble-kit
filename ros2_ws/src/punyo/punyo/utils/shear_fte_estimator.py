# Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.

from enum import Enum

import cv2
import math

from punyo.utils.contact_patch_estimator import ContactPatchEstimator
from punyo.utils.cv_utils import visualize_flow_arrows, force_arrow_visualize
from punyo.utils.flow_vis import *
from punyo.utils.raft_flow import RaftEstimator

PYR_SCALE = 0.5
CPE_DEPTH_OFFSET = 5
FLOW_WINDOW_SIZE = 50
FLOW_LEVELS = 30


class OPTICAL_FLOW_METHOD(Enum):
    FARNEBACK = 1
    RAFT = 2


class ShearFTEstimator:
    def __init__(self, method=OPTICAL_FLOW_METHOD.FARNEBACK, device="cuda", enable_cpe=True, debug=False):
        self._flow = None
        self._fx_scale: float = 1.0
        self._fy_scale: float = 1.0
        self._debug = debug
        self._enable_cpe = enable_cpe
        self._cpe = ContactPatchEstimator(CPE_DEPTH_OFFSET, debug=self._debug)

        if method == OPTICAL_FLOW_METHOD.RAFT:
            self.raft_estimator = RaftEstimator(device=device)

    def run(self,
            reference_gray_image,
            current_gray_image,
            reference_rgb_image,
            current_rgb_image,
            reference_depth_image,
            current_depth_image,
            method=OPTICAL_FLOW_METHOD.FARNEBACK):

        if method == OPTICAL_FLOW_METHOD.FARNEBACK:
            # Calculate the optical flow relative to the reference gray image
            self._flow = cv2.calcOpticalFlowFarneback(
                reference_gray_image,
                current_gray_image,
                self._flow,
                PYR_SCALE,
                FLOW_LEVELS,
                FLOW_WINDOW_SIZE,
                iterations=5,
                poly_n=7,
                poly_sigma=1.5,
                flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
            )

            # overlay optical flow arrows
            flow_arrows = visualize_flow_arrows(self._flow, current_rgb_image)
            # flow to color wheel
            flow_direction = flow_to_color(self._flow)

            cv2.imshow("FARNEBACK-flow_arrows", flow_arrows)
            cv2.imshow("FARNEBACK-flow_direction", flow_direction)
        elif method == OPTICAL_FLOW_METHOD.RAFT:
            # Calculate the optical flow relative to the reference rgb image
            self._flow = self.raft_estimator.calculate_raft_flow(reference_rgb_image, current_rgb_image)
            self._flow = self._flow[0].permute(1, 2, 0).cpu().detach().numpy()

            # overlay optical flow arrows
            flow_arrows = visualize_flow_arrows(self._flow, current_rgb_image)
            # flow to color wheel
            flow_direction = flow_to_color(self._flow)
            flow_direction = flow_direction[:, :, [2, 1, 0]] / 255.0

            cv2.imshow("RAFT-flow_arrows", flow_arrows)
            cv2.imshow("RAFT-flow_direction", flow_direction)
        else:
            raise Exception('Optical flow method was not recognized')

        current_depth_mask = None
        centroid = None

        if self._enable_cpe:
            current_depth_mask, centroid = self._cpe.generate_depth_mask(reference_depth_image, current_depth_image)

        # Calculate the force-torque for the x and y axes
        force_torque = self.calculate_force_torque(self._flow, current_depth_mask)

        # Update the flow image to include a single force arrow
        flow_img = force_arrow_visualize(flow_arrows, force_torque, centroid)

        if self._debug:
            if reference_depth_image is not None:
                cv2.imshow("s1. reference_depth_image", reference_depth_image)
            if current_depth_image is not None:
                cv2.imshow("s2. current_depth_image", current_depth_image)
            if current_depth_mask is not None:
                cv2.imshow("s3. current_depth_mask", current_depth_mask)
            cv2.imshow("s4. flow_image", flow_img)

        return force_torque, flow_img

    def calculate_force_torque(self, flow, current_depth_mask):
        """ Calculate the force torque wrench for the x and y axes"""

        # Optionally use the depth mask to only use those points in contact.
        masked_optical_flow = cv2.bitwise_and(flow, flow, mask=current_depth_mask)
        fx = self._fx_scale * np.mean(masked_optical_flow[:, :, 0])
        fy = self._fy_scale * np.mean(masked_optical_flow[:, :, 1])

        # for visualization, adding the max of the abs(magnitudes), f_x + f_y
        fx_max = self._fx_scale * np.max(masked_optical_flow[:, :, 0])
        fy_max = self._fy_scale * np.max(masked_optical_flow[:, :, 1])
        fx_min = self._fx_scale * np.min(masked_optical_flow[:, :, 0])
        fy_min = self._fy_scale * np.min(masked_optical_flow[:, :, 1])
        magnitude_max = math.sqrt(math.pow(max(fx_max, abs(fx_min)),2) + math.pow(max(fy_max, abs(fy_min)),2))

        # overloading the standard ft wrench with an additional value for viz
        ft = np.array([float(fx), float(fy), 0.0, 0.0, 0.0, 0.0, float(magnitude_max)])

        return ft
