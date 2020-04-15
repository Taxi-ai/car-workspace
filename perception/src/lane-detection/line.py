from collections import deque

import numpy as np
import cv2


# Define a class to receive the characteristics of each line detection
class Line:
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    def __init__(self, n):
        # was the line detected in the last iteration?
        self.detected = False
        # x values of the last n fits of the line, x = ay^2 + by + c
        self.n_xfitted = deque(maxlen=n)
        # x = ay^2 + by + c, for current detected line
        self.current_xfitted = None
        # average x values of the fitted line over the last n iterations
        self.bestx = None
        # polynomial coefficients of the last n iteration
        self.n_poly_coef = deque(maxlen=n)
        # polynomial coefficients averaged over the last n iterations
        self.best_poly_coef = None
        # polynomial coefficients for the most recent fit
        self.current_poly_coef = None
        # radius of curvature of the line in some units
        self.radius_of_curvature = None
        # distance in meters of vehicle center from the line
        self.line_base_pos = None
        # x values for detected line pixels
        self.current_x = None
        # y values for detected line pixels
        self.current_y = None
        # real world poly coefficients used to get real world radius
        self.real_cof = None

    def fit_polynomial(self, y):
        """
        calculate polynomial coefficient of line and update current x fitted & best x fitted & best_poly_coef
        :param y: y index which using to calculate it's x with poly
        """

        # Find our line poly coefficient.
        self.current_poly_coef = np.polyfit(self.current_y, self.current_x, 2)
        self.real_cof = np.polyfit(self.current_y * Line.ym_per_pix, self.current_x * Line.xm_per_pix, 2)

        # append current poly coefficient to poly coefficient deque
        self.n_poly_coef.append(self.current_poly_coef)

        # update best poly coefficient
        self.best_poly_coef = np.average(self.n_poly_coef, axis=0)

        self.current_xfitted = self.best_poly_coef[0] * y ** 2 + self.best_poly_coef[1] * y \
                               + self.best_poly_coef[2]

        self.n_xfitted.append(self.current_xfitted)
        self.bestx = np.average(self.n_xfitted, axis=0)

    def radius(self):
        """
        calculate radius of line
        """
        y = np.max(self.current_y) * Line.ym_per_pix
        r = ((1 + (2 * self.real_cof[0] * y + self.real_cof[1]) ** 2) ** 1.5) \
            / (np.abs(2 * self.real_cof[0]))

        self.radius_of_curvature = r

    def draw_line(self, img, y, color=(0, 0, 255), thickness=3):
        """
        Draw line using best x fitted on given img
        :param img: image which will draw line on it
        :param y: y index which used to draw line
        :param color: Tube (R,G,B) for line color, default = (0,0,255) == blue
        :param thickness: thickness of line, default = 3
        """
        # draw points needs to be int32 and transposed for cv2
        draw_points = np.asarray([self.bestx, y]).T.astype(np.int32)
        cv2.polylines(img, [draw_points], False, color, thickness=thickness)

    def color_pixel(self, img, color=(255, 255, 0)):
        """
        Color pixel belong to this line with given color
        :param img: image which will colored pixel of it
        :param color: Tube (R,G,B) for pixels color default = (255,0,0) == red
        """
        img[self.current_y, self.current_x] = color

    def car_offset(self, img_shape):
        """
        update line_base_pos which is distance in meters of vehicle center from the line
        :param img_shape: shape of frame
        :return: None
        """
        line_y = img_shape[0] - 1
        line_x = self.best_poly_coef[0] * line_y ** 2 + self.best_poly_coef[1] * line_y + self.best_poly_coef[2]

        self.line_base_pos = np.abs(line_x - (img_shape[1] / 2)) * Line.xm_per_pix