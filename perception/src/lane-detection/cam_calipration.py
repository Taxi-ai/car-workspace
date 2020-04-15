#!/usr/bin/env python3
import pickle
import sys
import argparse
import cv2
import numpy as np
import matplotlib.image as matimg
import os

from helper import save_pickle


def load_images(path):
    """
    :param path: Path to all calibration images
    :return: numpy array of all images
    """
    if not os.path.exists(path):
        print("Given path not exist!")
        sys.exit(1)

    print("Loading Images...")
    imags_names = os.listdir(path)
    imgs = []
    for img in imags_names:
        img = matimg.imread(path + '/' + img)
        img = np.resize(img, [720, 1280, 3])
        imgs.append(img)
    if len(imgs) < 10:
        print("Given oath contain less than 10 images! we need at least 10")
        sys.exit(1)

    imgs = np.array(imgs)
    print("Finished Loading Images..")
    return imgs


def find_chess_corners(imgs, nx, ny):
    """
    :param imgs: numpy array of images
    :param nx: number of chess board inside corners in row
    :param ny: number of chess board inside corners in column
    :return: list of lists, every inside list contain corners location of corresponding image
    """
    corners_list = []
    for i in range(imgs.shape[0]):
        img = imgs[i, :, :, :]
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
        if ret:
            corners_list.append(corners)

    return corners_list


def calibrate(path, nx=9, ny=6):
    """
    :param path: Path to all calibration images
    :param nx: number of chess board inside corners in row
    :param ny: number of chess board inside corners in column
    :return: calibration parameters [ret, mtx, dist, revecs, tvecs]
    """
    imgs = load_images(path)
    imgpoints = find_chess_corners(imgs, nx, ny)
    objps = []
    print("Start finding corners in Images...")
    for i in range(len(imgpoints)):
        p = np.zeros((nx * ny, 3), np.float32)
        p[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        objps.append(p)
    print("Calibrating your cam....")
    ret, mtx, dist, revecs, tvecs = cv2.calibrateCamera(objps, imgpoints, imgs[0].shape[0:2], None, None)
    return ret, mtx, dist, revecs, tvecs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", required=True,
                    help="calibration images path")
    ap.add_argument("-o", "--output_file",
                    help=" pickle file name contain [ret, mtx, dist, revecs, tvecs]", default="cam_calibration")
    ap.add_argument("-nx",
                    help="number of internal corner in row, default=9", default=9)
    ap.add_argument("-ny",
                    help="number of internal corner in column, default=6", default=6)
    args = vars(ap.parse_args())

    path = args['path']
    outpt = args['output_file']
    nx = args['nx']
    ny = args['ny']

    ret, mtx, dist, revecs, tvecs = calibrate(path, nx, ny)
    pickle_dict = {
        "ret": ret,
        "mtx": mtx,
        "dist": dist,
        "revecs": revecs,
        "tvecs": tvecs
    }
    print("Saving file...")
    save_pickle(pickle_dict,outpt)
    print("Finished!")