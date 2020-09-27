import numpy as np
import pyorbslam
import threading

from typing import Optional, Tuple


class MonocularTracker:
    """A monocular ORB-SLAM tracker."""

    # CONSTRUCTORS

    def __init__(self, *, image_size: Tuple[int, int], settings_file: str, use_viewer: bool = False, voc_file: str,
                 wait_till_ready: bool = False):
        """
        Construct a monocular ORB-SLAM tracker.

        :param image_size:          The image size of the monocular camera whose pose is to be tracked.
        :param settings_file:       The path to the file containing the settings to use for ORB-SLAM.
        :param use_viewer:          Whether or not to use ORB-SLAM's viewer (for debugging purposes).
        :param voc_file:            The path to the file containing the ORB vocabulary for ORB-SLAM.
        :param wait_till_ready:     Whether to block until the tracker is ready.
        """
        self.__image_size: Tuple[int, int] = image_size
        self.__settings_file: str = settings_file
        self.__use_viewer = use_viewer
        self.__voc_file: str = voc_file

        self.__should_terminate: bool = False

        self.__image: Optional[np.ndarray] = None
        self.__pose: Optional[np.ndarray] = None
        self.__timestamp: float = 0.0

        self.__lock = threading.Lock()
        self.__image_ready = threading.Condition(self.__lock)
        self.__pose_ready = threading.Condition(self.__lock)
        self.__tracker_ready = threading.Condition(self.__lock)
        self.__tracking_available: bool = False
        self.__tracking_required: bool = False

        self.__tracking_thread = threading.Thread(target=self.__process_tracking)
        self.__tracking_thread.start()

        # Block until the tracker is ready if requested.
        if wait_till_ready:
            with self.__lock:
                while not self.__tracking_available:
                    self.__tracker_ready.wait()

    # SPECIAL METHODS

    def __enter__(self):
        """No-op (needed to allow the ORB-SLAM wrapper's lifetime to be managed by a with statement)."""
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        """Destroy the ORB-SLAM wrapper at the end of the with statement that's used to manage its lifetime."""
        self.__should_terminate = True
        self.__tracking_thread.join()

    # PUBLIC METHODS

    def estimate_pose(self, image: np.ndarray) -> Optional[np.ndarray]:
        """
        Estimate the pose of the camera at the point at which the specified image was captured.

        .. note::
            Since this is a tracker rather than a relocaliser, internal state will be used when estimating the pose.
            As such, only sequential images should be passed to this method, or pose estimation won't work.

        :param image:   The image.
        :return:        The estimated pose of the camera at the point at which the image was captured.
        """
        with self.__lock:
            if self.__tracking_available and not self.__should_terminate:
                # Pass the image to the tracking thread.
                self.__image = image
                self.__tracking_required = True
                self.__image_ready.notify()

                # Wait for the tracking thread to estimate the pose.
                while self.__tracking_required:
                    self.__pose_ready.wait(0.1)
                    if self.__should_terminate:
                        return None

                return self.__pose
            else:
                # If tracking is not yet available, early out.
                return None

    def is_ready(self):
        """
        Get whether or not tracking is available yet.

        :return:    True, if tracking is available, or False otherwise.
        """
        with self.__lock:
            return self.__tracking_available

    # PRIVATE METHODS

    def __process_tracking(self):
        """
        Process tracking requests on a separate thread.
        """
        # Initialise ORB-SLAM.
        system: pyorbslam.System = pyorbslam.System(
            self.__voc_file, self.__settings_file, pyorbslam.MONOCULAR, self.__use_viewer
        )

        # Allocate a suitably-sized OpenCV image that can be passed to C++.
        image: pyorbslam.CVMat3b = pyorbslam.CVMat3b.zeros(*self.__image_size)

        with self.__lock:
            # Advertise that tracking is now available.
            self.__tracking_available = True
            self.__tracker_ready.notify()

            # While the tracker should not terminate:
            while not self.__should_terminate:
                # Wait for a tracking request.
                while not self.__tracking_required:
                    self.__image_ready.wait(0.1)
                    if self.__should_terminate:
                        return

                # Process the tracking request.
                np.copyto(np.array(image, copy=False), self.__image)
                pose: pyorbslam.CVMat1d = system.track_monocular(image, self.__timestamp)
                self.__pose = np.array(pose)
                self.__timestamp += 0.1
                self.__tracking_required = False
                self.__pose_ready.notify()
