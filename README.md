# Vins_gpu_ros2
humble version

# About
This is fork from [VINS-Fusion-gpu](https://github.com/pjrambo/VINS-Fusion-gpu) for OpenCV 4.10 and compatible to ceres-solver 2.3.0. And now work with ros2 humble.

VINS-Fusion-gpu is a version of [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) with GPU acceleration.


# Dependencies
- [Ceres Solver](http://ceres-solver.org/installation.html)
- [ROS](http://wiki.ros.org/ROS/Installation)
- [OpenCV](https://opencv.org)
- [OpenCV bridge](https://github.com/ros-perception/vision_opencv)


# Ceres Solver

[Installation](http://ceres-solver.org/installation.html).

[Download](http://ceres-solver.org/ceres-solver-2.0.0.tar.gz) latest stable release. Test it on ceres-solver-1.14 and ceres-solver-2.0.

#### Dependencies

```
sudo apt-get install -y libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
```

#### Build Ceres
```
#check this for sfm part: https://docs.opencv.org/4.x/db/db8/tutorial_sfm_installation.html

echo "------------------------------------"
echo "** Install requirement (1/3)"
echo "------------------------------------"
sudo apt-get install libabsl-dev
sudo apt-get install libeigen3-dev libgflags-dev libgoogle-glog-dev
# static library
sudo apt-get install libsuitesparse-dev

echo "------------------------------------"
echo "** Cloning repo (2/3)"
echo "------------------------------------"
git clone https://ceres-solver.googlesource.com/ceres-solver

cd ceres-solver
git submodule update --init --recursive

# here build the absl and gtest in submodule
cd right-path-of-submodule
cmake ..
make 
sudo make install

# build the ceres
mkdir build 
cd build
# for shared library maybe
cmake .. 
echo "------------------------------------"
echo "** BUilding (3/3)"
echo "------------------------------------"
make -j$(nproc)
sudo make install
```

# Build the opencv
```

version="4.10.0"
folder="opencv-${version}"


echo "------------------------------------"
echo "** Install requirement (1/5)"
echo "------------------------------------"
# sudo apt-get update
# sudo apt install libopenblas-dev libopenblas-base libatlas-base-dev liblapacke-dev
# sudo apt install libjpeg-dev libpng-dev libtiff-dev
# sudo apt install libavcodec-dev libavformat-dev libswscale-dev
# sudo apt install libv4l-dev v4l-utils
# sudo apt install libxvidcore-dev libx264-dev
# sudo apt install libgtk-3-dev
# sudo apt install protobuf-compiler
# sudo apt install python3-dev python3-venv python3-numpy python3-wheel python3-setuptools
# sudo apt install tesseract-ocr

# sudo mkdir /usr/include/openblas
# sudo cp /usr/include/lapacke.h /usr/include/openblas/
# sudo cp /usr/include/lapacke_mangling.h /usr/include/openblas/
# sudo cp /usr/include/lapacke_config.h /usr/include/openblas/
# sudo cp /usr/include/lapacke_utils.h  /usr/include/openblas/
# sudo cp /usr/include/x86_64-linux-gnu/cblas*.h /usr/include/


echo "------------------------------------"
echo "** Create Python Environment (2/5)"
echo "------------------------------------"
# python3 -m venv .env --prompt opencv_cuda
# source .env/bin/activate
# pip install wheel numpy tesseract
# deactivate



echo "------------------------------------"
echo "** Download opencv "${version}" (3/5)"
echo "------------------------------------"
mkdir $folder
cd ${folder}
# curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
# curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
# unzip opencv-${version}.zip
# unzip opencv_contrib-${version}.zip
# rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/


echo "------------------------------------"
echo "** Build opencv "${version}" (4/5)"
echo "------------------------------------"
mkdir release
cd release/

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export OPENCV_VERSION=4.10.0
cmake \
-D CMAKE_BUILD_TYPE=Release \
-D OPENCV_VERSION=${OPENCV_VERSION} \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=1 \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_ARCH_BIN=8.9 \
-D Atlas_CBLAS_INCLUDE_DIR=/usr/lib/x86_64-linux-gnu/ \
-D OpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/ \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules \
-D PYTHON3_EXECUTABLE=/home/alexmanson/altivision/vins_ws/env/.env/bin/python \
-D PYTHON_LIBRARIES=/home/alexmanson/altivision/vins_ws/env/.env/lib/python3.10/site-packages \
-D BUILD_opencv_python3=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON ..


echo "------------------------------------"
echo "** Install opencv "${version}" (5/5)"
echo "------------------------------------"

make -j$(nproc)
sudo make install

echo "** OpenCV ${version} installed successfully in ${PWD}/install"
```

# Build the package itsel
```
sudo apt install ros-humble-nav-msgs

# please help me fill out

rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble
```



# Build
Build is same as [VINS-Fusion-gpu](https://github.com/pjrambo/VINS-Fusion-gpu#2-build-vins-fusion) and [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion#2-build-vins-fusion).

#### Dependencies
```
sudo apt-get install ros-melodic-tf ros-melodic-image-transport
```

#### Build
Clone the repository and catkin_make:
```
cd ~/catkin_ws/src
git clone https://github.com/IOdissey/VINS-Fusion-GPU.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```


# Usage
In the config file, there are two parameters for gpu acceleration.

GPU for GoodFeaturesToTrack (0 - off, 1 - on) 
```
use_gpu: 1
```

GPU for OpticalFlowPyrLK (0 - off, 1 - on) 
```
use_gpu_acc_flow: 1
```

If your GPU resources is limitted or you want to use GPU for other computaion. You can set
```
use_gpu: 1
use_gpu_acc_flow: 0
```

If your other application do not require much GPU resources (recommanded)
```
use_gpu: 1
use_gpu_acc_flow: 1
```

There are two parameters for config OpticalFlowPyrLK (pyramid level and window size):
```
lk_n: 3
lk_size: 21
```
