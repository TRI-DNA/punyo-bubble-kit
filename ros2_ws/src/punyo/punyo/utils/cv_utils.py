# Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.

import cv2
import numpy as np


def visualize_flow_arrows(flow, img, scale=1, step=20):
    """ Given a flow, add an overlay of force vectors to the image. """
    h, w = img.shape[0], img.shape[1]
    flag = False
    color = (20, 255, 255)  # BGR

    arrows_img = np.zeros_like(img)
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])

    # Add the arrows, skipping every *step* pixels
    for i in range(0, mag.shape[0], step):
        for j in range(0, mag.shape[1], step):
            mags = scale * mag[i, j]
            if int(mags):
                ndx = min(i + int(mags * np.sin(ang[i, j])), h)
                ndy = min(j + int(mags * np.cos(ang[i, j])), w)
                pt1 = (j, i)
                pt2 = (max(ndy, 0), max(ndx, 0))
                arrows_img = cv2.arrowedLine(
                    arrows_img,
                    pt1,
                    pt2,
                    color,
                    2,
                    tipLength=0.25,
                )
                flag = True
    if flag:
        if len(img.shape) == 3:
            # Just want to overlay the arrows
            img2gray = cv2.cvtColor(arrows_img, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(img2gray, 1, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(mask)
            masked_source = cv2.bitwise_and(img, img, mask=mask_inv)
            img = cv2.addWeighted(arrows_img, 1.0, masked_source, 1.0, 0)
        else:
            img = cv2.add(img, arrows_img)

    return img


def force_arrow_visualize(img, f, centroid, scale=40):
    h, w = img.shape[0], img.shape[1]

    if centroid is None:
        center = (int(w / 2.0), int(h / 2.0))
    else:
        center = (centroid[0], centroid[1])

    shear_tip = np.around(np.array([center[0] + f[0] * scale, center[1] + f[1] * scale])).astype(int)

    img = cv2.arrowedLine(
        img,
        pt1=center,
        pt2=tuple(shear_tip),
        color=(250, 250, 250),
        thickness=2,
        tipLength=0.5,
    )
    normal_tip = np.around(
        np.array([center[0], center[1] + f[2] * scale])
    ).astype(int)

    img = cv2.arrowedLine(
        img,
        pt1=center,
        pt2=tuple(normal_tip),
        color=(50, 255, 50),
        thickness=2,
        tipLength=0.5,
    )
    return img
