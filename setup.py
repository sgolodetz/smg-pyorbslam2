import skbuild

with open("README.md", "r") as fh:
    long_description = fh.read()

skbuild.setup(
    name="smg-pyorbslam2",
    version="0.0.1",
    author="Stuart Golodetz",
    author_email="stuart.golodetz@cs.ox.ac.uk",
    description="Python bindings for ORB-SLAM2",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/sgolodetz/smg-pyorbslam2",
    packages=["smg.pyorbslam2", "smg.pyorbslam2.cpp"],
    cmake_install_dir="smg/pyorbslam2/cpp",
    include_package_data=True,
    install_requires=[
        "numpy",
        "smg-pyopencv"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='==3.7.*',
)
