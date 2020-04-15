#!/usr/bin/env python3
import sys

import cv2
import numpy as np

from helper import hist, threshold_channel, channel_Isolate


def sobel(img, orient='x', thresh_min=20, thresh_max=100):
    """
    apply sobel filter in (orient) direction
    :param img: RGB or Gray image
    :param orient: orientation of Sobel filter [x or y]
    :param thresh_min: minimum threshold of edges
    :param thresh_max: maximum threshold of edges
    :return: binary images contain detected edges
    """
    if len(img.shape) > 2 and img.shape[2] != 1:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    else:
        gray = img

    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0)  # cv2.CV_64f == np array of type 64 bit float
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
    sobel = np.absolute(sobel)
    sobel = np.uint8((255 * sobel) / np.max(sobel))
    scaled_sobel = np.zeros_like(sobel)
    scaled_sobel[(sobel > thresh_min) & (sobel <= thresh_max)] = 255
    return scaled_sobel


def sv_edges(img, s_thresh=(100, 200), v_thresh=(180, 200)):
    """
    Detect edges using s and v channels and l channel in lab color space
    :param img: RGB image
    :param s_thresh: s channel threshold, (minimum  threshold, maximum threshold)
    :param v_thresh: v channel threshold, (minimum  threshold, maximum threshold)
    :return:  (s , v, b) detected edges
    """
    if len(img.shape) != 2 and img.shape[2] != 3:
        print("Image should be rgb!")
        sys.exit(1)

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # s channel edges
    s = hsv[:, :, 1]
    s_binary_output = np.zeros_like(s)
    s_binary_output[(s > s_thresh[0]) & (s <= s_thresh[1])] = 255
    # v channel edges
    v = hsv[:, :, 2]
    v_binary_output = np.zeros_like(v)
    v_binary_output[(v > v_thresh[0]) & (v <= v_thresh[1])] = 255

    lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    b = lab[:, :, 2]
    b_binary_output = np.zeros_like(v)
    b_binary_output[(b > 140) & (b <= 255)] = 255
    return s_binary_output, v_binary_output, b_binary_output


def apply_edge_detection(img):
    """
    apply edge detection on given image
    :param img: source image
    :return: binary image with edges detected
    """



    red_threshed = threshold_channel(channel_Isolate(img, 'R'), (220, 255))
    V_threshed = threshold_channel(channel_Isolate(img, 'V'), (220, 255))
    HSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    yellow = cv2.inRange(HSV, (20, 100, 100), (50, 255, 255))

    sensitivity_1 = 68
    white = cv2.inRange(HSV, (0, 0, 255 - sensitivity_1), (255, 20, 255))

    sensitivity_2 = 60
    HSL = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    white_2 = cv2.inRange(HSL, (0, 255 - sensitivity_2, 0), (255, 255, sensitivity_2))

    white_3 = cv2.inRange(img, (200, 200, 200), (255, 255, 255))

    bit_layer = red_threshed | V_threshed | yellow | white | white_2 | white_3
    return bit_layer


def sliding_window(img, nwindows, width, minpix=50, draw=False):
    """
    apply sliding window on img to detect pixels belong to lane lines
    :param img: binary warped image
    :param width: window width
    :param nwindows: number of sliding windows
    :param minpix: number if minimum number of pixels = 1 in window to recenter window
    :param draw: if True draw sliding window on out image
    :return: leftx, lefty, rightx, righty, if draw is True return out_img too
    """
    histogram = hist(img)

    margin = width // 2
    height = img.shape[0] // nwindows
    midpoint = img.shape[1] // 2

    left_side = histogram[0:midpoint]
    right_side = histogram[midpoint:]

    left_x_base = np.argmax(left_side)
    right_x_base = np.argmax(right_side) + midpoint

    nonzero = img.nonzero()
    nonzerox = nonzero[1]
    nonzeroy = nonzero[0]

    leftx_current = left_x_base
    rightx_current = right_x_base

    if draw:
        out_img = np.dstack((img, img, img))

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = img.shape[0] - (height * (window + 1))
        win_y_high = img.shape[0] - (height * window)

        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        if draw:
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)),
                          (0, 0, 255), 3)
            cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)),
                          (0, 0, 255), 3)

        good_left_inds = ((nonzeroy >= win_y_low) &
                          (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) &
                          (nonzerox < win_xleft_high)).nonzero()[0]

        good_right_inds = ((nonzeroy >= win_y_low) &
                           (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) &
                           (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    if draw:
        return leftx, lefty, rightx, righty, out_img

    return leftx, lefty, rightx, righty




def search_around_poly(line, warped_img, margin=100):
    """
    Search around polynomial to update lane lines curve
    :param line: object of Line class
    :param warped_img: warped image of current frame
    :param margin: margin value for search area ( left amd right )
    :return: x and y index for pixels belong to lane line
    """
    nonzero = warped_img.nonzero()
    nonzerox = nonzero[1]
    nonzeroy = nonzero[0]

    lane_inds = ((nonzerox > line.best_poly_coef[0] * (nonzeroy ** 2) + line.best_poly_coef[1] * nonzeroy +
                  line.best_poly_coef[2] - margin) &
                 (nonzerox < line.best_poly_coef[0] * (nonzeroy ** 2) + line.best_poly_coef[1] * nonzeroy
                  + line.best_poly_coef[2] + margin))

    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]
    return x, y