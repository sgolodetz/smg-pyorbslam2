import pyorbslam

from pyorbslam import CVMat3b


def main():
    voc_file: str = "C:/orbslam/Vocabulary/ORBvoc.txt"
    settings_file: str = "D:/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0005_sync/orb_slam/settings.yaml"
    system: pyorbslam.System = pyorbslam.System(voc_file, settings_file, pyorbslam.MONOCULAR, False)

    im: CVMat3b = CVMat3b.zeros(480, 640)
    system.track_monocular(im, 0.0)


if __name__ == "__main__":
    main()
