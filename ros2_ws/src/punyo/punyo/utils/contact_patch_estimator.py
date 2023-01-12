# Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.

import numpy as np
import cv2


class ContactPatchEstimator:

    def __init__(self, depth_offset: int, debug=False):
        self._depth_offset = depth_offset
        self._blob_size_threshold = 150
        self._debug = debug
        self.fgbg = cv2.createBackgroundSubtractorMOG2()

    def compute_diff(self, image1, image2):
        diff_image = np.subtract(image1.astype(np.int16), image2.astype(np.int16))
        diff_image[diff_image < 0] = 0
        return diff_image.astype(np.uint8)

    def generate_depth_mask(self, reference_depth_image, current_depth):
        if reference_depth_image is None or current_depth is None:
            return None, None

        offset_image = self.compute_diff(current_depth, reference_depth_image)

        plane_range_y = np.linspace(
            self._depth_offset,
            1 * self._depth_offset,
            np.size(offset_image, 1),
        )

        plane_offset = np.tile(
            plane_range_y, (np.size(offset_image, 0), 1)
        ).astype(np.uint8)

        offset_image_after_plane = self.compute_diff(offset_image, plane_offset)

        # ret, mask = cv2.threshold(offset_image_after_plane, 0, 1, cv2.THRESH_BINARY| cv2.THRESH_OTSU)
        ret, mask = cv2.threshold(offset_image_after_plane, 0, 255, cv2.THRESH_BINARY)

        (nb_components, output, stats, centroids) = cv2.connectedComponentsWithStats(mask, connectivity=8)

        sizes = stats[1:, -1]
        nb_components = nb_components - 1

        mask_fixed = np.zeros((mask.shape))

        for i in range(0, nb_components):
            if sizes[i] >= self._blob_size_threshold:
                mask_fixed[output == i + 1] = 255
        mask_fixed = mask_fixed.astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 2))
        res = cv2.morphologyEx(mask_fixed, cv2.MORPH_OPEN, kernel)
        depth_mask = res.astype(np.uint8)

        if self._debug:
            cv2.imshow("c1. offset_image (diff)", offset_image)
            cv2.imshow("c2. offset_image_after_plane", offset_image_after_plane)
            cv2.imshow("c3. mask (threshold)", mask)
            cv2.imshow("c4. mask_fixed (blob_size)", mask_fixed)
            cv2.imshow("c5. depth mask", depth_mask)

        # fgmask = self.fgbg.apply(current_depth)
        # cv2.imshow("fgmask", fgmask)

        centroid = self.get_centroid(depth_mask)

        return depth_mask, centroid

    def get_centroid(self, current_depth_mask):
        M = cv2.moments(current_depth_mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroid = np.array([cX, cY])
        else:
            centroid = None

        return centroid
