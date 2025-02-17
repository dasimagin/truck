# ACHTUNG!
# Platform arm64 means nvidia jetson arm64.
# Image may be not compatible with other arm machines.

FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 AS truck-base-arm64

ENV FLAGS="-O3 -Wall -march=armv8.2-a+simd+crypto+predres -mtune=cortex-a57"

### INSTALL NVIDIA

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
    nvidia-cuda-dev \
    nvidia-cudnn8-dev \
    nvidia-tensorrt-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

### INSTALL JETSON STATS

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends python3-pip \
    && pip3 install --no-cache-dir -U pip \
    && pip3 install --no-cache-dir -U jetson-stats \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

FROM --platform=linux/amd64 ubuntu:jammy-20240911.1 AS truck-base-amd64

ENV FLAGS="-O3 -Wall"

FROM truck-base-${TARGETARCH} AS truck-common

WORKDIR /tmp

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# For image build we use GCC (CUDA and ROS2 compatible)
ENV GCC_VERSION=11
ENV CC="gcc-${GCC_VERSION}"
ENV CXX="g++-${GCC_VERSION}"
ENV CFLAGS="${FLAGS}"
ENV CXXFLAGS="${FLAGS}"

RUN echo "Build info:" \
    && echo "  TARGETARCH=${TARGETARCH}" \
    && echo "  CC=${CC}" \
    && echo "  CXX=${CXX}" \
    && echo "  CFLAGS=${CFLAGS}" \
    && echo "  CXXFLAGS=${CXXFLAGS}"

### INSTALL COMMON PKGS

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    apt-transport-https \
    apt-utils \
    ca-certificates \
    gcc-${GCC_VERSION} \
    g++-${GCC_VERSION} \
    cmake\
    curl \
    git \
    gnupg2 \
    libceres-dev \
    libmpfr-dev \
    libboost-dev \
    libpython3-dev \
    make \
    software-properties-common \
    gnupg \
    python3 \
    python3-dev \
    python3-distutils \
    python3-pip \
    python3-setuptools \
    tar \
    wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### INSTALL GSTREAMER

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    gstreamer1.0-tools \
    gstreamer1.0-alsa \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-rtsp \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### INSTALL EIGEN

ARG EIGEN_VERSION="3.4.0"

RUN wget -qO - https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz | tar -xz \
    && cd eigen-${EIGEN_VERSION} && mkdir -p build && cd build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### PREPARE FOR OPENCV

RUN apt-get update -yq && \
    apt-get install -yq --no-install-recommends \
    gfortran \
    file \
    libatlas-base-dev \
    libjpeg-dev \
    liblapack-dev \
    liblapacke-dev \
    libopenblas-dev \
    libpng-dev \
    libtbb-dev \
    libtbb2 \
    pkg-config \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV OPENCV_VERSION=4.10.0

### PREPARE FOR TORCH

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gfortran \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    libjpeg-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* apt-get clean

### PREPARE FOR REALSENSE2

RUN apt-get update -yq \
    && apt-get install -yq --no-install-recommends \
    ca-certificates \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libomp-dev \
    pkg-config \
    sudo \
    udev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV LIBRS_VERSION=2.56.2

FROM --platform=linux/arm64 truck-common AS truck-cuda-arm64

### INSTALL OPENCV

RUN wget -qO - https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && wget -qO - https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build \
    && OPENCV_MODULES=(core calib3d imgproc imgcodecs ccalib ximgproc \
    cudev cudaarithm cudacodec cudafilters cudaimgproc) \
    && cmake .. \
    -DBUILD_LIST=$(echo ${OPENCV_MODULES[*]} | tr ' ' ',') \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DWITH_GTK=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_java=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCUDA_ARCH_BIN=5.3,6.2,7.2 \
    -DCUDA_ARCH_PTX= \
    -DCUDA_FAST_MATH=ON \
    -DCUDNN_INCLUDE_DIR=/usr/include \
    -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -DWITH_EIGEN=ON \
    -DENABLE_NEON=ON \
    -DOPENCV_DNN_CUDA=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
    -DWITH_CUBLAS=ON \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DWITH_GSTREAMER=ON \
    -DWITH_LIBV4L=ON \
    -DWITH_OPENGL=ON \
    -DWITH_OPENCL=OFF \
    -DWITH_IPP=OFF \
    -DWITH_TBB=ON \
    -DWITH_TIFF=OFF \
    -DWITH_OPENEXR=OFF \
    -DWITH_JASPER=OFF \
    -DWITH_WITH_OPENJPEG=OFF \
    -DWITH_WEBP=OFF \
    -DWITH_IMGCODEC_HDR=OFF \
    -DWITH_IMGCODEC_SUNRASTER=OFF \
    -DWITH_IMGCODEC_PXM=OFF \
    -DWITH_IMGCODEC_PFM=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    && make -j$(nproc) install && rm -rf /tmp/*

### INSTALL PYTORCH

RUN pip3 install --no-cache-dir \
    Cython \
    wheel \
    numpy \
    pillow

ENV PYTORCH_WHL="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
ENV PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/${PYTORCH_WHL}"

RUN wget --no-check-certificate -qO ${PYTORCH_WHL} ${PYTORCH_URL} \
    && pip3 install --no-cache-dir ${PYTORCH_WHL} \
    && rm -rf /tmp/*

ENV PYTORCH_PATH="/usr/local/lib/python3.10/dist-packages/torch"
ENV LD_LIBRARY_PATH="${PYTORCH_PATH}/lib:${LD_LIBRARY_PATH}"

### INSTALL REALSENSE2

RUN wget -qO - https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBRS_VERSION}.tar.gz | tar -xz \
    && cd librealsense-${LIBRS_VERSION} \
    && bash scripts/setup_udev_rules.sh \
    && mkdir -p build && cd build \
    && cmake .. \
    -DBUILD_EXAMPLES=false \
    -DCMAKE_BUILD_TYPE=release \
    -DFORCE_RSUSB_BACKEND=false \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_WITH_OPENMP=true \
    -DBUILD_PYTHON_BINDINGS=false \
    -DBUILD_WITH_TM2=false \
    && make -j$(($(nproc)-1)) install \
    && rm -rf /tmp/*

FROM --platform=linux/amd64 truck-common AS truck-cuda-amd64

### INSTALL OPENCV

RUN wget -qO - https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && wget -qO - https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build \
    && OPENCV_MODULES=(core calib3d imgcodecs ccalib ximgproc) \
    && cmake .. \
    -DBUILD_LIST=$(echo ${OPENCV_MODULES[*]} | tr ' '  ',') \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DWITH_GTK=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_java=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -DWITH_EIGEN=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
    -DWITH_GSTREAMER=ON \
    -DWITH_LIBV4L=ON \
    -DWITH_OPENCL=ON \
    -DWITH_IPP=OFF \
    -DWITH_TBB=ON \
    -DWITH_PNG=ON \
    -DBUILD_PNG=OFF \
    -DWITH_TIFF=OFF \
    -DBUILD_TIFF=OFF \
    -DWITH_WEBP=OFF \
    -DBUILD_WEBP=OFF \
    -DWITH_OPENJPEG=OFF \
    -DBUILD_OPENJPEG=OFF \
    -DWITH_JASPER=OFF \
    -DBUILD_JASPER=OFF \
    -DWITH_OPENEXR=OFF \
    -DBUILD_OPENEXR=OFF \
    -DWITH_IMGCODEC_HDR=OFF \
    -DWITH_IMGCODEC_SUNRASTER=OFF \
    -DWITH_IMGCODEC_PXM=OFF \
    -DWITH_IMGCODEC_PFM=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    && make -j$(nproc) install && rm -rf /tmp/*

### INSTALL PYTORCH

ENV TORCH_VERSION=2.5.0

RUN pip3 install --no-cache-dir \
    torch==${TORCH_VERSION}

ENV PYTORCH_PATH="/usr/local/lib/python3.10/dist-packages/torch"
ENV LD_LIBRARY_PATH="${PYTORCH_PATH}/lib:${LD_LIBRARY_PATH}"

### INSTALL REALSENSE2

RUN apt-get update -yq \
    && apt-get install -yq --no-install-recommends \
    ca-certificates \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libomp-dev \
    pkg-config \
    sudo \
    udev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBRS_VERSION}.tar.gz | tar -xz \
    && cd librealsense-${LIBRS_VERSION} \
    && bash scripts/setup_udev_rules.sh \
    && mkdir -p build && cd build \
    && cmake .. \
    -DBUILD_EXAMPLES=false \
    -DCMAKE_BUILD_TYPE=release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=false \
    -DBUILD_WITH_OPENMP=false \
    -DBUILD_PYTHON_BINDINGS=false \
    -DBUILD_WITH_TM2=false \
    && make -j$(($(nproc)-1)) install \
    && rm -rf /tmp/*

FROM truck-cuda-${TARGETARCH} AS truck-ros

ENV ROS_VERSION=2
ENV ROS_DISTRO=iron
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

RUN wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \

    ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
    libpcl-dev \
    locales \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-rosinstall-generator \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV ROS_TMP=/tmp/${ROS_DISTRO}

RUN mkdir -p ${ROS_ROOT} \
    && mkdir -p ${ROS_TMP} && cd ${ROS_TMP} \
    && rosinstall_generator \
    --rosdistro ${ROS_DISTRO} \
    --exclude librealsense2 libpointmatcher libnabo \
    --deps \
    compressed_depth_image_transport \
    compressed_image_transport \
    cv_bridge \
    foxglove_bridge \
    foxglove_msgs \
    image_geometry \
    image_transport \
    imu_filter_madgwick \
    imu_complementary_filter\
    geometry2 \
    launch_yaml \
    laser_geometry \
    pcl_conversions \
    realsense2_camera \
    rmw_cyclonedds_cpp \
    robot_localization \
    ros_base \
    sensor_msgs \
    sensor_msgs_py \
    std_msgs \
    vision_opencv \
    visualization_msgs \
    > ${ROS_ROOT}/ros2.rosinstall \
    && vcs import ${ROS_TMP} < ${ROS_ROOT}/ros2.rosinstall

RUN apt-get update -q \
    && rosdep init \
    && rosdep update \
    && rosdep install -qy --ignore-src  \
    --rosdistro ${ROS_DISTRO} \
    --from-paths ${ROS_TMP} \
    --skip-keys fastcdr \
    --skip-keys rti-connext-dds-6.0.1 \
    --skip-keys urdfdom_headers \
    --skip-keys librealsense2 \
    --skip-keys libopencv-dev \
    --skip-keys libopencv-contrib-dev \
    --skip-keys libopencv-imgproc-dev \
    --skip-keys libpcl-dev \
    --skip-keys python3-opencv \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# Pay attention that we disable pedantic warnings here!
# The reason is that foxglove_bridge has such warnings.
RUN cd ${ROS_TMP} \
    && colcon build \
    --merge-install \
    --install-base ${ROS_ROOT} \
    --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /tmp/*

ENV SLLIDAR_COMMIT=a64c75979d52ada7f646bc9505f6133c69e195ee

RUN git clone https://github.com/Slamtec/sllidar_ros2.git \
    && cd sllidar_ros2 \
    && git checkout ${SLLIDAR_COMMIT} \
    && source ${ROS_ROOT}/setup.bash \
    && colcon build \
    --merge-install \
    --install-base ${ROS_ROOT} \
    --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /tmp/*

FROM truck-ros AS truck-dev

ARG TARGETPLATFORM  # Automatically set by docker buildx

ENV MEDIAMTX_VERSION="1.9.3"

RUN declare -A map \
    && map["linux/amd64"]="linux_amd64" \
    && map["linux/arm64"]="linux_arm64v8" \
    && wget -qO - https://github.com/aler9/mediamtx/releases/download/v${MEDIAMTX_VERSION}/mediamtx_v${MEDIAMTX_VERSION}_${map[$TARGETPLATFORM]}.tar.gz | tar -xz -C /usr/bin mediamtx

### INSTALL LIBPOINTMATCHER

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    libtbb-dev \
    libproj-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ARG LIBNABO_VERSION="1.1.2"

RUN wget -qO - https://github.com/ethz-asl/libnabo/archive/refs/tags/${LIBNABO_VERSION}.tar.gz | tar -xz \
    && cd libnabo-${LIBNABO_VERSION} && mkdir -p build && cd build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DLIBNABO_BUILD_TESTS=OFF \
    -DLIBNABO_BUILD_EXAMPLES=OFF \
    -DLIBNABO_BUILD_PYTHON=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

ARG LIBPOINTMATCHER_VERSION="1.4.3"

RUN wget -qO - https://github.com/ethz-asl/libpointmatcher/archive/refs/tags/${LIBPOINTMATCHER_VERSION}.tar.gz | tar -xz \
    && cd libpointmatcher-${LIBPOINTMATCHER_VERSION} && mkdir -p build && cd build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DPOINTMATCHER_BUILD_EXAMPLES=OFF \
    -DPOINTMATCHER_BUILD_EVALUATIONS=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL CGAL

ARG CGAL_VERSION="6.0"

RUN wget -qO - https://github.com/CGAL/cgal/archive/refs/tags/v${CGAL_VERSION}.tar.gz | tar -xz \
    && cd cgal-${CGAL_VERSION} && mkdir -p build && cd build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL G2O

ARG G2O_HASH="b1ba729aa569267e179fa2e237db0b3ad5169e2e"

RUN git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o && git checkout ${G2O_HASH} \
    && CMAKE_VERSION=$(cmake --version | grep -oP "(?<=cmake version )\d+\.\d+") \
    && cp cmake_modules/FindG2O.cmake "/usr/share/cmake-${CMAKE_VERSION}/Modules/FindG2O.cmake" \
    && mkdir -p build && cd build \
    && cmake .. \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_USE_OPENGL=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL GTSAM

ARG GTSAM_VERSION="4.1.1"

RUN wget -qO - https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz | tar -xz \
    && cd gtsam-${GTSAM_VERSION} && mkdir -p build && cd build \
    && cmake .. \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL LIVOX SDK2

ARG LIVOX_VERSION="1.2.5"

RUN wget -qO - https://github.com/Livox-SDK/Livox-SDK2/archive/refs/tags/v${LIVOX_VERSION}.tar.gz | tar -xz \
    && cd Livox-SDK2-${LIVOX_VERSION} && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install \
    && rm -rf /tmp/*

### INSTALL RAPIDJSON

ARG RAPIDJSON_VERSION="1.1.0"

RUN wget -qO - https://github.com/Tencent/rapidjson/archive/refs/tags/v${RAPIDJSON_VERSION}.tar.gz | tar -xz \
    && cd rapidjson-${RAPIDJSON_VERSION} && mkdir build && cd build \
    && cmake .. -DRAPIDJSON_BUILD_DOC=OFF -DRAPIDJSON_BUILD_EXAMPLES=OFF -DRAPIDJSON_BUILD_TESTS=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/*

### INSTALL DEV PKGS

COPY requirements.txt /tmp/requirements.txt

RUN python3 -m pip install --no-cache-dir --ignore-installed -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# Install clang for tooling, but use gcc as default compiler
ENV CLANG_VERSION=20
ENV CFLAGS="${FLAGS} -std=c17"
ENV CXXFLAGS="${FLAGS} -std=c++20"

RUN wget https://apt.llvm.org/llvm.sh \
    && chmod +x llvm.sh \
    && sudo ./llvm.sh ${CLANG_VERSION}


RUN echo "Build info:" \
    && echo "  TARGETARCH=${TARGETARCH}" \
    && echo "  CC=${CC}" \
    && echo "  CXX=${CXX}" \
    && echo "  CFLAGS=${CFLAGS}" \
    && echo "  CXXFLAGS=${CXXFLAGS}"

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
    clang-${CLANG_VERSION} \
    clang-format-${CLANG_VERSION} \
    clang-tidy-${CLANG_VERSION} \
    htop \
    libpugixml-dev \
    libfmt-dev \
    less \
    lldb-${CLANG_VERSION} \
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# Install zsh
RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends zsh \
    && chsh -s $(which zsh) $(whoami) \
    && sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# Install fzf
ENV FZF_VERSION=0.56.0
RUN wget -qO - "https://github.com/junegunn/fzf/releases/download/v${FZF_VERSION}/fzf-${FZF_VERSION}-linux_$(dpkg --print-architecture).tar.gz" | sudo tar -xz -C /usr/local/bin

RUN printf "export ROS_ROOT=${ROS_ROOT}\n" >> /root/.zshrc \
    && printf "export ROS_DISTRO=${ROS_DISTRO}\n" >> /root/.zshrc \
    && printf "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}\n" >> /root/.zshrc \
    && printf "export RCUTILS_LOGGING_BUFFERED_STREAM=1\n" >> /root/.zshrc \
    && printf "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}:{time}] {message}'\n" >> /root/.zshrc \
    && printf "source ${ROS_ROOT}/setup.zsh\n" >> /root/.zshrc \
    && printf "export CC='${CC}'\n" >> /root/.zshrc \
    && printf "export CXX='${CXX}'\n" >> /root/.zshrc \
    && printf "export CFLAGS='${CFLAGS}'\n" >> /root/.zshrc \
    && printf "export CXXFLAGS='${CXXFLAGS}'\n" >> /root/.zshrc \
    && printf "export CMAKE_BUILD_TYPE=Release\n" >> /root/.zshrc \
    && printf "export TRUCK_SIMULATION=false\n" >> /root/.zshrc \
    && printf "source <(fzf --zsh)\n" >> /root/.zshrc \
    && ln -sf /usr/bin/clang-format-${CLANG_VERSION} /usr/bin/clang-format

### SETUP ENTRYPOINT

WORKDIR /truck
ENTRYPOINT ["/bin/zsh", "-lc"]
CMD ["trap : TERM INT; sleep infinity & wait"]
