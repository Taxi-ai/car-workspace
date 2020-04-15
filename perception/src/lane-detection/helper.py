#!/usr/bin/env python3
import os
import pickle
import sys

import cv2
import numpy as np


class Error(Exception):
    pass


def write_text(img, text, pos, font_scale=1, color=(255, 255, 255)):
    """
    Write text on given image at the given position
    :param img: src image
    :param text: text to write
    :param pos: pos of text on the image
    :param font_scale: size of font, default =1
    :param color: color of text, default= (255,255,255) white
    :return: img with text on it
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_type = 2

    cv2.putText(img, text, pos, font, font_scale, color, line_type)
    return img

def perspective_tf(src, dst):
    """
    apply perspective transform on img using src and dist points
    :param src: list of src point
    :param dst: list of destination point which will transform src to it
    :return: M => transformed transform matrix, Minv => inverse transform matrix
    """
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    return M, Minv


def warped_img(img, M):
    """
    apply perspective transform on given img using given matrix
    :param img: image want to apply transform on it
    :param M: perspective TF matrix
    :return: transformed image
    """
    img_size = (img.shape[1], img.shape[0])
    warped = cv2.warpPerspective(img, M, img_size)
    return warped


def save_pickle(dic, name, path=None):
    """
    Save dictionary as pickle file
    :param dic: dictionary want to save
    :param name: name of saved pickle file
    :param path: save location path, if not given save in current file directory
    :return: None
    """
    if path and not os.path.isfile(path):
        print("Given path: ", path, " Not Exist!")
        sys.exit(1)
    if not path:
        path = ""
    pickle_out = open(path + name + ".pickle", "wb")
    pickle.dump(dic, pickle_out)
    pickle_out.close()
    print("Pickle file: " + name + " saved successfully")


def hist(img):
    """
    calculate histogram of given img
    :param img: gray img
    :return: histogram of given img
    """

    if max(np.max(img, axis=0)) == 255:
        img = img / 255
    img_shape = img.shape
    bottom_half = img[img_shape[0] // 2: img_shape[1] // 2, :]
    histogram = np.sum(bottom_half, axis=0)

    return histogram


def channel_Isolate(image, channel):
    # Takes in only RBG images
    if channel == 'R':
        return image[:, :, 0]

    elif channel == 'G':
        return image[:, :, 1]

    elif channel == 'B':
        return image[:, :, 2]

    elif channel == 'H':
        HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        return HSV[:, :, 0]

    elif channel == 'S':
        HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        return HSV[:, :, 1]

    elif channel == 'V':
        HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        return HSV[:, :, 2]

    elif channel == 'L':
        HLS = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        return HLS[:, :, 1]

    elif channel == 'Cb':
        YCrCb = cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)
        return YCrCb[:, :, 2]

    elif channel == 'U':
        LUV = cv2.cvtColor(image, cv2.COLOR_RGB2Lab)
        return LUV[:, :, 2]

    else:
        raise ValueError("Channel must be either R, G, B, H, S, V, L, Cb, U")


def threshold_channel(channel, thresh):
    retval, binary = cv2.threshold(channel.astype('uint8'), thresh[0], thresh[1], cv2.THRESH_BINARY)
    return binary