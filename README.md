# smg-pyorbslam2

This Python package provides Python bindings for ORB-SLAM2.

It is a submodule of [smglib](https://github.com/sgolodetz/smglib), the open-source Python framework associated with our drone research in the [Cyber-Physical Systems](https://www.cs.ox.ac.uk/activities/cyberphysical/) group at the University of Oxford.

### Development Installation (as part of smglib)

Note: Please read the [top-level README](https://github.com/sgolodetz/smglib/blob/master/README.md) for smglib before following these instructions.

1. Download and extract a copy of Eigen (if you have not already done so), and set (at a system level, not within the terminal) the `SMGLIB_EIGEN_INCLUDE_DIR` environment variable to the top-level Eigen directory, e.g.

```
C:/Users/<your user>/Downloads/eigen-3.4.0
```

2. Also set (at a system level) the `SMGLIB_ORBSLAM2_ROOT_DIR` to `C:/orbslam2` and the `SMGLIB_Pangolin_ROOT_DIR` to `C:/Pangolin` (if you use other directories, you'll need to adjust the instructions further down accordingly).

3. Open the terminal.

4. Build Pangolin using Visual Studio.

   i. Clone our fork of Pangolin into `C:/Pangolin`:

   ```
   git clone git@github.com:sgolodetz/Pangolin.git C:/Pangolin
   ```

   ii. Change to the `C:/Pangolin` directory.

   iii. Check out the `winchanges` branch of Pangolin using `git checkout winchanges`.

   iv. Configure Pangolin using CMake, with `C:/Pangolin/build` as the build directory. Set `CMAKE_INSTALL_PREFIX` to `C:/Pangolin/install`. Then build and install Pangolin using Visual Studio (in both `Debug` and `Release` modes).

5. Build ORB-SLAM2 using Visual Studio.

   i. Clone our fork of ORB-SLAM2 into `C:/orbslam2`:

   ```
   git clone git@github.com:sgolodetz/ORB_SLAM2.git C:/orbslam2
   ```

   ii. Change to the `C:/orbslam2` directory.

   iii. Check out the `winchanges` branch of ORB-SLAM2 using `git checkout winchanges`.

   iv. Configure DBoW2 (in `C:/orbslam2/Thirdparty/DBoW2`) using CMake, with `C:/orbslam2/Thirdparty/DBoW2/build` as the build directory. Set relevant variables as per the box below. Then build (but don't install) DBoW2 using Visual Studio (in both `Debug` and `Release` modes).

   ```
   CMAKE_INSTALL_PREFIX=C:/orbslam2/Thirdparty/DBoW2/install
   OpenCV_DIR=<the SMGLIB_OPENCV_DIR environment variable value>
   ```

   v. Configure g2o (in `C:/orbslam2/Thirdparty/g2o`) using CMake, with `C:/orbslam2/Thirdparty/g2o/build` as the build directory. Set relevant variables as per the box below. Then build (but don't install) g2o using Visual Studio (in both `Debug` and `Release` modes).

   ```
   CMAKE_INSTALL_PREFIX=C:/orbslam2/Thirdparty/g2o/install
   EIGEN3_INCLUDE_DIR=<the SMGLIB_EIGEN_INCLUDE_DIR environment variable value>
   ```

   Note:
   - If you're unlucky, you may need to modify the optimisation options for the `g2o` project when building in `Release` mode to prevent Visual Studio from hanging during the build. We found `Custom` optimisation with `/Ob2 /Oi /Ot /Oy` to work, but YMMV.

   vi. Configure ORB-SLAM2 itself using CMake, with `C:/orbslam2/build` as the build directory. Setting relevant variables as per the box below, build (but don't install) ORB-SLAM2 using Visual Studio (in both `Debug` and `Release` modes).

   ```
   CMAKE_BUILD_TYPE=<whichever of Debug or Release you're building at the time>
   CMAKE_INSTALL_PREFIX=C:/orbslam2/install
   EIGEN3_INCLUDE_DIR=<the SMGLIB_EIGEN_INCLUDE_DIR environment variable value>
   OpenCV_DIR=<the SMGLIB_OPENCV_DIR environment variable value>
   ```

   Notes:
   - Don't forget to reconfigure with a different `CMAKE_BUILD_TYPE` each time, and to do a full rebuild. (Yes, this is hideous.)
   - If you're unlucky, you may need to set the `/bigobj` flag on `Optimizer.cc` when building in `Debug` mode to work around a compiler limit. (This is even more hideous - sorry.)

5. Back in the terminal, change to the `<root>/smg-pyorbslam2` directory.

6. Check out the `master` branch of `smg-pyorbslam2`.

7. Activate the Conda environment using ```conda activate smglib```.

8. Run `pip install -e .` at the terminal.

### Publications

If you build on this framework for your research, please cite the following paper:
```
@inproceedings{Golodetz2022TR,
author = {Stuart Golodetz and Madhu Vankadari* and Aluna Everitt* and Sangyun Shin* and Andrew Markham and Niki Trigoni},
title = {{Real-Time Hybrid Mapping of Populated Indoor Scenes using a Low-Cost Monocular UAV}},
booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
month = {October},
year = {2022}
}
```

### Acknowledgements

This work was supported by Amazon Web Services via the [Oxford-Singapore Human-Machine Collaboration Programme](https://www.mpls.ox.ac.uk/innovation-and-business-partnerships/human-machine-collaboration/human-machine-collaboration-programme-oxford-research-pillar), and by UKRI as part of the [ACE-OPS](https://gtr.ukri.org/projects?ref=EP%2FS030832%2F1) grant. We would also like to thank [Graham Taylor](https://www.biology.ox.ac.uk/people/professor-graham-taylor) for the use of the Wytham Flight Lab, [Philip Torr](https://eng.ox.ac.uk/people/philip-torr/) for the use of an Asus ZenFone AR, and [Tommaso Cavallari](https://uk.linkedin.com/in/tcavallari) for implementing TangoCapture.
