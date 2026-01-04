# Vins_gpu_ros2
humble version


## Ceres
Change the ceres version to use GPU to accelerate ceres_solver, currently using version 2.2.0

Install [ceres-2.2.0](http://ceres-solver.org/ceres-solver-2.2.0.tar.gz)
### Install Dependencies
```bash
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev

```

### build
```bash
tar zxf ceres-solver-2.2.0.tar.gz
cd ceres-solver-2.2.0
mkdir build && cd build

cmake \
-DCMAKE_BUILD_TYPE=Release \
-DBUILD_TESTING=OFF \
-DBUILD_EXAMPLES=OFF \
-DUSE_CUDA=ON \
-DSUITESPARSE=ON \
-DLAPACK=ON \
-DEIGENSPARSE=ON \
..

make -j$(nproc)

sudo make install
```

## vins_octomap
### Install octomap from source
[octomap-1.9.7](https://github.com/OctoMap/octomap/releases/tag/v1.9.7)

Build Only octomap (Core Library)

```bash
cd octomap
mkdir build
cd build
cmake ..
make
```

Build Full Package (octomap + octovis)

```bash
mkdir build && cd build
cmake ..
make
```
### Other dependencies

```bash
sudo apt install ros-humble-octomap-msgs
```

### rviz2 octomap visualization

```bash
sudo apt-get install ros-${ROS_DISTRO}-octomap-rviz-plugins
```

from source
```bash
cd ~/work_ws
git clone https://github.com/OctoMap/octomap_rviz_plugins.git -b ros2
colcon build --packages-select octomap_rviz_plugins
source install/setup.bash
rviz2
```

