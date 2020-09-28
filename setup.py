import setuptools

# with open("README.md", "r") as fh:
#     long_description = fh.read()

setuptools.setup(
    name="smg-pyorbslam",
    version="0.0.1",
    author="Stuart Golodetz",
    author_email="sgolodetz@gmail.com",
    description="Python bindings for ORB-SLAM",
    long_description="",  #long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    packages=["pyorbslam"],
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='==3.7.*',
)
