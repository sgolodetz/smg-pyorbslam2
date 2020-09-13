import cv2
import numpy as np
import pyorbslam

from pyorbslam import CVMat1d, CVMat1f, CVMat3b
from typing import List


def load_colour_image(filename: str) -> CVMat3b:
    img: np.ndarray = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    mat: CVMat3b = CVMat3b.zeros(img.shape[0], img.shape[1])
    np.copyto(np.array(mat, copy=False), img)
    return mat


def load_depth_image(filename: str) -> CVMat1f:
    img: np.ndarray = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    mat: CVMat1f = CVMat1f.zeros(img.shape[0], img.shape[1])
    np.copyto(np.array(mat, copy=False), img)
    return mat


def main():
    np.set_printoptions(suppress=True)

    voc_file: str = "C:/orbslam/Vocabulary/ORBvoc.txt"
    settings_file: str = "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/settings.yaml"
    colour_images: List[CVMat3b] = [load_colour_image(filename) for filename in [
        "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-000000.color.png",
        "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-000001.color.png",
    ]]
    depth_images: List[CVMat1f] = [load_depth_image(filename) for filename in [
        "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-000000.depth.png",
        "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-000001.depth.png",
    ]]
    # pyorbslam.f(im)
    system: pyorbslam.System = pyorbslam.System(voc_file, settings_file, pyorbslam.RGBD, False)

    timestamp: float = 0.0
    for colour_image, depth_image in zip(colour_images, depth_images):
        pose: CVMat1d = system.track_rgbd(colour_image, depth_image, timestamp)
        print(np.array(pose, copy=False))
        timestamp += 0.1


if __name__ == "__main__":
    main()
