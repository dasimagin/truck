FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

ENV ROS_VERSION=2
ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

WORKDIR /tmp

### COMMON BASE

ENV CLANG_VERSION=12

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        apt-transport-https \
        apt-utils \
        build-essential \
        ca-certificates \
        clang-format-${CLANG_VERSION} \
        clang-tidy-${CLANG_VERSION} \
        cmake\
        curl \
        git \
        gnupg2 \
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

ENV FLAGS="-O3 -ffast-math -Wall"
ENV CFLAGS="${FLAGS}"
ENV CXXFLAGS="${FLAGS}"

### INSTALL OPENCV

ENV OPENCV_VERSION="4.5.0"

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        gfortran \
        file \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libavresample-dev \
        libdc1394-22-dev \
        libeigen3-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer1.0-dev \
        libjpeg-dev \
        libjpeg8-dev \
        libjpeg-turbo8-dev \
        liblapack-dev \
        liblapacke-dev \
        libopenblas-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtbb2 \
        libtesseract-dev \
        libtiff-dev \
        libv4l-dev \
        libxine2-dev \
        libxvidcore-dev \
        libx264-dev \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && wget -qO - https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build \
    && OPENCV_MODULES=(core calib3d features2d flann highgui imgcodecs video videoio stitching photo \
        aruco bgsegm ccalib optflow rgbd sfm stereo xfeatures2d ximgproc xphoto) \
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
        -DBUILD_TIFF=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
    && make -j$(nproc) install && rm -rf /tmp/*

# ### INSTALL REALSENSE2

ARG LIBRS_VERSION="2.51.1"

RUN apt-get update -yq \
    && apt-get install -yq --no-install-recommends \
        ca-certificates \
        libssl-dev \
        libusb-1.0-0-dev \
        libusb-1.0-0 \
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

### INSTALL PYTORCH

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libopenblas-dev \
        libopenmpi-dev \
        libomp-dev \
        gfortran \
        libjpeg-dev \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* apt-get clean

ARG TORCH_VERSION=1.12.0
ARG TORCHVISION_VERSION=0.13.0

RUN pip3 install --no-cache-dir \
    torch==${TORCH_VERSION} torchvision==${TORCHVISION_VERSION} \
        --extra-index-url https://download.pytorch.org/whl/cpu \
    Cython \
    wheel \
    numpy \
    pillow

# ### INSTALL RTAB-MAP

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        libeigen3-dev \
        libyaml-cpp-dev \
        libboost-dev \
        libtbb-dev \
        libsqlite3-dev \
        libpcl-dev \
        libproj-dev \
        libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ARG G2O_HASH="b1ba729aa569267e179fa2e237db0b3ad5169e2e"

RUN git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o && git checkout ${G2O_HASH} \
    && mkdir -p build && cd build \
    && cmake .. \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DG2O_BUILD_APPS=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_USE_OPENGL=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

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

ARG LIBNABO_VERSION="1.0.7"

RUN wget -qO - https://github.com/ethz-asl/libnabo/archive/refs/tags/${LIBNABO_VERSION}.tar.gz | tar -xz \
    && cd libnabo-${LIBNABO_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DLIBNABO_BUILD_TESTS=OFF \
        -DLIBNABO_BUILD_EXAMPLES=OFF \
        -DLIBNABO_BUILD_PYTHON=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

ARG LIBPOINTMATCHER_VERSION="1.3.1"

RUN wget -qO - https://github.com/ethz-asl/libpointmatcher/archive/refs/tags/${LIBPOINTMATCHER_VERSION}.tar.gz | tar -xz \
    && cd libpointmatcher-${LIBPOINTMATCHER_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DPOINTMATCHER_BUILD_EXAMPLES=OFF \
        -DPOINTMATCHER_BUILD_EVALUATIONS=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

# Use ROS release repo, version is not fixed!
ARG RTAB_MAP_BRANCH="release/${ROS_DISTRO}/rtabmap"

RUN git clone https://github.com/introlab/rtabmap-release.git \
    && cd rtabmap-release && git checkout ${RTAB_MAP_BRANCH} \
    && mkdir -p build && cd build \
    && cmake .. \
        -DBUILD_APP=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL ROS2

RUN wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        gazebo11 \
        gazebo11-common \
        gazebo11-plugin-base \
        imagemagick \
        libgazebo11-dev \
        libjansson-dev \
        libasio-dev \
        libboost-dev \
        libtinyxml-dev \
        locales \
        mercurial \
        python3-bson \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-numpy \
        python3-pytest-cov \
        python3-rosdep \
        python3-vcstool \
        python3-rosinstall-generator \
        libtinyxml2-dev \
        libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN pip3 install --no-cache-dir -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    setuptools

ENV ROS_TMP=/tmp/${ROS_DISTRO}

RUN mkdir -p ${ROS_ROOT} \
    && mkdir -p ${ROS_TMP} && cd ${ROS_TMP} \
    && rosinstall_generator \
    --rosdistro ${ROS_DISTRO} \
    --exclude librealsense2 rtabmap libg2o libpointmatcher libnabo \
    --deps \
        ament_cmake_clang_format \
        compressed_depth_image_transport \
        compressed_image_transport \
        cv_bridge \
        foxglove_msgs \
        image_geometry \
        image_transport \
        image_rotate \
        gazebo_dev \
        gazebo_plugins \
        gazebo_ros \
        geometry2 \
        joy \
        launch_xml \
        launch_yaml \
        pcl_conversions \
        robot_localization \
        realsense2_camera \
        rmw_cyclonedds_cpp \
        ros_base \
        rosbridge_suite \
        rtabmap_ros \
        sensor_msgs \
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
        --skip-keys clang-format \
        --skip-keys clang-tidy \
        --skip-keys fastcdr \
        --skip-keys rti-connext-dds-5.3.1 \
        --skip-keys urdfdom_headers \
        --skip-keys libg2o \
        --skip-keys librealsense2 \
        --skip-keys libopencv-dev \
        --skip-keys libopencv-contrib-dev \
        --skip-keys libopencv-imgproc-dev \
        --skip-keys python3-opencv \
        --skip-keys rtabmap \
        --skip-keys gazebo11 \
        --skip-keys gazebo11-common \
        --skip-keys gazebo11-plugin-base \
        --skip-keys libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN cd ${ROS_TMP} \
    && colcon build \
        --merge-install \
        --install-base ${ROS_ROOT} \
        --cmake-args -DBUILD_TESTING=OFF \ 
        --catkin-skip-building-tests \
    && rm -rf /tmp/*

RUN printf "export ROS_ROOT=${ROS_ROOT}\n" >> /root/.bashrc \
    && printf "export ROS_DISTRO=${ROS_DISTRO}\n" >> /root/.bashrc \
    && printf "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}\n" >> /root/.bashrc \
    && printf "source ${ROS_ROOT}/setup.bash\n" >> /root/.bashrc

ENV SLLIDAR_COMMIT=4bbeb1a61d08812ea8465eecd0f27030ccff92c4

COPY patch/sllidar.patch /tmp/sllidar.patch

RUN git clone https://github.com/Slamtec/sllidar_ros2.git \
    && cd sllidar_ros2 \
    && git checkout ${SLLIDAR_COMMIT} \
    && git apply /tmp/sllidar.patch \
    && source ${ROS_ROOT}/setup.bash \
    && colcon build \
        --merge-install \
        --install-base ${ROS_ROOT} \
    && rm -rf /tmp/*

### INSTALL DEV PKGS

COPY requirements.txt /tmp/requirements.txt

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        bluez \
        file \
        htop \
        lldb-${CLANG_VERSION} \
        ssh \
    && python3 -m pip install --no-cache-dir -U -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt \
    && rm -rf /var/lib/apt/lists/* && apt-get clean


ENV CC="gcc-9"
ENV CXX="g++-9"

ENV CFLAGS="${FLAGS} -std=c17"
ENV CXXFLAGS="${FLAGS} -std=c++2a"

RUN printf "export CC='${CC}'\n" >> /root/.bashrc \
    && printf "export CXX='${CXX}'\n" >> /root/.bashrc \
    && printf "export CFLAGS='${CFLAGS}'\n" >> /root/.bashrc \
    && printf "export CXXFLAGS='${CXXFLAGS}'\n" >> /root/.bashrc \
    && printf "export RCUTILS_LOGGING_BUFFERED_STREAM=1\n" >> /root/.bashrc \
    && printf "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}:{time}] {message}'\n" >> /root/.bashrc \
    && printf "PermitRootLogin yes\nPort 2222" >> /etc/ssh/sshd_config \
    && echo 'root:root' | chpasswd

### SETUP ENTRYPOINT

COPY /entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
WORKDIR /truck
