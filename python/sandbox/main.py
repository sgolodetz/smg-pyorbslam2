import pyorbslam


def main():
    voc_file: str = ""
    settings_file: str = ""
    system: pyorbslam.System = pyorbslam.System(voc_file, settings_file, pyorbslam.RGBD, False)
    pass


if __name__ == "__main__":
    main()
