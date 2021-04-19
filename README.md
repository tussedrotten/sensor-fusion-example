# sensor-fusion-example
Example for experiments with sensor fusion and factor graphs.

Contains the following data:
- GPS
- IMU
- Barometer
- Camera feature tracks

The navigation sensor data is stored on [NavLab](http://www.navlab.net) format.

Thanks to very good collegues at FFI for allowing this data to be shared!


## Executables:
- [plot_raw_data](plot_raw_data.cpp): Plots the raw data.
- [gps_imu_batch](gps_imu_batch.cpp): GPS-IMU batch fusion.
- [gps_imu_fixed_lag](gps_imu_fixed_lag.cpp): GPS-IMU fixed lag fusion.
- [gps_imu_isam](gps_imu_isam.cpp): GPS-IMU fusion with ISAM2.


## Install dependencies
Detailed procedure for Ubuntu when starting for scratch.
Pick the pieces you need.

### Essentials: Install compiler, cmake, curl and git
```bash
sudo apt install -y \
  build-essential \
  cmake \
  curl \
  git \
  wget
```

### Install Eigen
We can install a sufficient version of Eigen with `apt`. In addition, we install blas and lapack.

```bash
sudo apt install -y \
  libblas-dev \
  liblapack-dev \
  libeigen3-dev
```

### Install Sophus
```bash
# Clone the repository (download the code) from GitHub.
git clone --depth 1 https://github.com/strasdat/Sophus.git
# Configure the project
# -S: source folder
# -B: build folder (will be created automatically)
cmake -S Sophus -B Sophus/build \
 -DCMAKE_BUILD_TYPE=Release \
 -DBUILD_EXAMPLES=OFF \
 -DBUILD_TESTS=OFF
# Copy the files to /usr/local/...
sudo cmake --build Sophus/build --target install
sudo rm -rf Sophus ~/.cmake/packages/Sophus
```

#### Install GeographicLib
We are also installing GeographicLib from source, but instead of cloning via `git` we are downloading the source code using `curl`.
Using the pipe, we are immediately extracting the downloaded `tar.gz` without it hanging around. It will take some time.

```bash
curl -fL# https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.51.tar.gz \
| tar -zx

cmake -S GeographicLib-1.51 -B GeographicLib-1.51/build -DCMAKE_BUILD_TYPE=Release
# Copy the files to /usr/local/...
sudo cmake --build GeographicLib-1.51/build --config Release --target install
# Delete the downloaded files
sudo rm -rf GeographicLib-1.51
```

### Install GTSAM
GTSAM has some additional dependencies that we must install before trying to compile it.

```bash
sudo apt install -y \
  libboost-all-dev \
  libtbb2 \
  libtbb-dev
```
Now we can go on.
```bash
git clone --depth 1 https://github.com/borglab/gtsam.git
cmake -S gtsam -B gtsam/build \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_EIGEN_MKL=OFF
cmake --build gtsam/build -- -j$(nproc)
sudo cmake --build gtsam/build --target install
rm -rf gtsam
```

### Install OpenCV
#### Install dependencies
```bash
# Install compiler
sudo apt-get update && sudo apt-get install -y \
  build-essential

# Install required
sudo apt install -y \
  cmake \
  cmake-qt-gui \
  git \
  libavcodec-dev \
  libavformat-dev \
  libgtk2.0-dev \
  libswscale-dev \
  pkg-config

# Install boost
sudo apt install -y \
  libboost-all-dev

# Install optional
sudo apt install -y \
  libdc1394-22-dev \
  libjpeg-dev \
  libpng-dev \
  libtbb2 \
  libtbb-dev \
  libtiff-dev \
  libvtk7-dev \
  mesa-utils \
  python3-dev \
  python3-numpy \
  qt5-default
  
# Install very optional
sudo apt install -y \
  libcanberra-gtk-module \
  libcanberra-gtk3-module
```

#### Compile OpenCV
```bash
# Convenience variable
# If variable OpenCV_VERSION is not set, set it to 4.0.1
tag=${OpenCV_VERSION:-4.0.1}

# Download opencv sources
git clone -b ${tag} --depth 1 https://github.com/opencv/opencv.git opencv-${tag}
git clone -b ${tag} --depth 1 https://github.com/opencv/opencv_contrib.git opencv_contrib-${tag}

mkdir opencv-${tag}/build
cd $_  # $_ is a special variable set to last arg of last command.

# Build OpenCV
cmake .. \
-DCPACK_MONOLITHIC_INSTALL=ON \
-DCMAKE_BUILD_TYPE=Release \
-DBUILD_DOCS=OFF \
-DBUILD_EXAMPLES=OFF \
-DBUILD_JAVA=OFF \
-DBUILD_PROTOBUF=ON \
-DBUILD_TBB=ON \
-DBUILD_TESTS=OFF \
-DBUILD_PERF_TESTS=OFF \
-DOPENCV_ENABLE_NONFREE=ON \
-DOPENCV_EXTRA_MODULES_PATH="../../opencv_contrib-${tag}/modules/" \
-DWITH_CUDA=OFF \
-DWITH_GDAL=ON \
-DWITH_PROTOBUF=ON \
-DPROTOBUF_UPDATE_FILES=OFF \
-DWITH_QT=ON \
-DBUILD_opencv_{java,js,python}=OFF \
-DBUILD_opencv_python2=OFF \
-DBUILD_opencv_python3=ON \
-DBUILD_opencv{\
bgsegm,bioinspired,ccalib,cnn_3dobj,cvv,datasets,dnn_objdetect,dnns_easily_fooled,dpm,face,freetype,\
fuzzy,hdf,hfs,img_hash,line_descriptor,matlab,optflow,ovis,phase_unwrapping,plot,reg,rgbd,saliency,sfm,shape,stereo,\
structured_light,superres,surface_matching,text,tracking,videostab}=OFF \
-DBUILD_opencv_cuda{bgsegm,codec,filters,legacy,objdetect,stereo}=OFF \
-DBUILD_opencv_cudev=OFF 

cmake --build . --config release -- -j $(nproc) -Wno-cpp
sudo cmake --build . --target install

cd ../../
rm -rf opencv*
```

### Install Python libraries
Used for plotting.

```bash
sudo apt-get install python-matplotlib python2.7-dev
```
