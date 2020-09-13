import cv2
import numpy as np
import pyorbslam

from pyorbslam import CVMat1d, CVMat1f, CVMat3b


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
    use_viewer: bool = True
    system: pyorbslam.System = pyorbslam.System(voc_file, settings_file, pyorbslam.RGBD, use_viewer)

    timestamp: float = 0.0
    for idx in range(100):
        colour_image: CVMat3b = load_colour_image(
            f"D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-{idx:06d}.color.png"
        )
        depth_image: CVMat1f = load_depth_image(
            f"D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/frame-{idx:06d}.depth.png"
        )
        pose: CVMat1d = system.track_rgbd(colour_image, depth_image, timestamp)
        print(np.array(pose, copy=False))
        cv2.waitKey(1000)
        timestamp += 0.1


if __name__ == "__main__":
    main()
