#!/usr/bin/env python3
import sys

import cv2
import numpy as np

from helper import hist, warped_img
from line import Line


class LaneDetection:

    def __init__(self, M, Minv, leftLine, rightLine):
        self.M = M
        self.Minv = Minv

        self.img = None
        self.threshold_img = None
        self.warped = None
        self.out_img = None
        self.leftLine = leftLine
        self.rightLine = rightLine

        self.ploty = None
        self.distance = 0
        self.turn_dir = "keep"  # take thre values keep, right, left

    def __call__(self, img):
        self.img = img  # RGB image
        self.apply_edge_detection()
        self.warped = warped_img(self.threshold_img, self.M)

        # if we have line poly from past frames, we just search around it in current frame
        if self.leftLine.detected and self.rightLine.detected:
            leftx, lefty = self.search_around_poly(self.leftLine)
            rightx, righty = self.search_around_poly(self.rightLine)

        # if we don't have line poly, search using sliding window
        else:
            leftx, lefty, rightx, righty, sliding = self.sliding_window(
                nwindows=9, width=100)
        if self.ploty is None:
            # will used for plotting line, find x fitted
            self.ploty = np.linspace(
                self.warped.shape[0] // 4, self.warped.shape[0] - 1, self.warped.shape[0])

        print("left x = " + str(len(leftx)) + "right x = " + str(len(rightx)))
        # check if at least 100 pixels detected as line
        if len(leftx) > 100 and len(rightx) > 100:
            self.leftLine.detected = True
            self.rightLine.detected = True

            self.leftLine.current_x = leftx
            self.leftLine.current_y = lefty

            self.rightLine.current_x = rightx
            self.rightLine.current_y = righty

            self.leftLine.fit_polynomial(self.ploty)
            self.rightLine.fit_polynomial(self.ploty)

            # make sure that detected lines is the lane lines by checking distance bettween them
            line_y = self.warped.shape[0] - 1
            left_x = self.leftLine.best_poly_coef[0] * line_y ** 2 + \
                self.leftLine.best_poly_coef[1] * \
                line_y + self.leftLine.best_poly_coef[2]

            right_x = self.rightLine.best_poly_coef[0] * line_y ** 2 + \
                self.rightLine.best_poly_coef[1] * \
                line_y + self.leftLine.best_poly_coef[2]
            dist = np.abs(left_x-right_x) * Line.xm_per_pix

            if dist < 0.6 or dist > 1.3:
                print("Wrong lane detected, distance = ", dist)
                self.leftLine.detected = False
                self.rightLine.detected = False

            else:
                print("right dist = " + str(dist))

        else:
            print("Line not detected in this frame ")
            # make detected flag fslse
            self.leftLine.detected = False
            self.rightLine.detected = False

        self.get_distance()
        if self.distance > 0:
            self.turn_dir = "left"
        elif self.distance < 0:
            self.turn_dir = "right"
        else:
            self.turn_dir = "keep"

    def drawLanes(self):
        if self.warped is None or self.leftLine.current_y is None:
            print("Can't draw lane lines before detect it!")
            return

        self.out_img = np.zeros_like(self.warped)
        self.out_img = np.dstack((self.out_img, self.out_img, self.out_img))

        # color pixels belong to lane lines
        self.leftLine.color_pixel(self.out_img, (255, 0, 0))
        self.rightLine.color_pixel(self.out_img, (255, 0, 0))

        # fit green triangle in area between lane lines
        pts_left = np.array(
            [np.transpose(np.vstack([self.leftLine.bestx, self.ploty]))])
        pts_right = np.array(
            [np.flipud(np.transpose(np.vstack([self.rightLine.bestx, self.ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(self.out_img, np.int_([pts]), (0, 255, 0))

        # return image to normal view from bird view
        out_img_undit = warped_img(self.out_img, self.Minv)

        self.img = cv2.addWeighted(out_img_undit, 0.5, self.img, 1, 0)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)

    def get_distance(self):
        """
        update distance value which is the displacment of car from lane center
        """
        # to make sure that we updated poly coef
        if self.leftLine.best_poly_coef is not None:
            # calculate Alignment ( how much car away from center between Lane lines
            # distance from left line
            self.leftLine.car_offset(self.warped.shape)
            # distance from right line
            self.rightLine.car_offset(self.warped.shape)

            self.distance = round(
                self.leftLine.line_base_pos - self.rightLine.line_base_pos, 2)
        else:
            self.distance = 0

    def apply_edge_detection(self):
        """
        apply edge detection on given image
        :param img: source image
        :return: binary image with edges detected
        """

        THRES_MIN = 127
        THRES_MAX = 255
        gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
        gray = cv2.blur(gray, (5, 5))

        _, self.threshold_img = cv2.threshold(
            gray, THRES_MIN, THRES_MAX, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    def sliding_window(self, nwindows, width, minpix=50, draw=False):
        """
        apply sliding window on img to detect pixels belong to lane lines
        :param nwindows: number of sliding windows
        :param width: window width
        :param minpix: number if minimum number of pixels = 1 in window to recenter window
        :param draw: if True draw sliding window on out image
        :return: leftx, lefty, rightx, righty, if draw is True return out_img too
        """
        histogram = hist(self.warped)

        margin = width // 2
        height = self.warped.shape[0] // nwindows
        midpoint = self.warped.shape[1] // 2

        # TODO check if this shift work fine or not!
        # we shift midpoint with 50
        shift = 100
        left_side = histogram[0:midpoint-shift]
        right_side = histogram[midpoint+shift:]

        left_x_base = np.argmax(left_side)
        right_x_base = np.argmax(right_side) + midpoint + shift

        nonzero = self.warped.nonzero()
        nonzerox = nonzero[1]
        nonzeroy = nonzero[0]

        leftx_current = left_x_base
        rightx_current = right_x_base

        if draw:
            out_img = np.dstack((self.warped, self.warped, self.warped))

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = self.warped.shape[0] - (height * (window + 1))
            win_y_high = self.warped.shape[0] - (height * window)

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

    def search_around_poly(self, line, margin=50):
        """
        Search around polynomial to update lane lines curve
        :param line: object of Line class
        :param margin: margin value for search area ( left amd right )
        :return: x and y index for pixels belong to lane line
        """
        nonzero = self.warped.nonzero()
        nonzerox = nonzero[1]
        nonzeroy = nonzero[0]

        lane_inds = ((nonzerox > line.best_poly_coef[0] * (nonzeroy ** 2) + line.best_poly_coef[1] * nonzeroy +
                      line.best_poly_coef[2] - margin) &
                     (nonzerox < line.best_poly_coef[0] * (nonzeroy ** 2) + line.best_poly_coef[1] * nonzeroy
                      + line.best_poly_coef[2] + margin))

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]
        return x, y
