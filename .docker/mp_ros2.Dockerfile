# Preserve rosdep installs separate from source. Maintains caches as long as rosdeps do not change.
ARG ROS_DISTRO=jazzy
FROM alpine:latest AS package-manifests
COPY . /motion_planner
RUN find /motion_planner -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /motion_planner -type d -empty -delete
FROM ros:${ROS_DISTRO} AS motion_planner
ENV ROS_DISTRO=jazzy
ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]
ARG CPU_ARCH=x86_64

# ---------- base tools ----------
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    ca-certificates \
    libfmt-dev \
    curl \
    gnupg \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ---------- core ROS / system deps ----------
RUN apt-get update && apt-get install -y \
    libgmock-dev \
    python3-pip \
    libglfw3-dev \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-urdfdom-py \
    libxi6 \
    libxkbcommon0 \
    && rm -rf /var/lib/apt/lists/*
# ---------- Mesa (CPU OpenGL, RViz, Qt) ----------
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt-get update && apt-get install -y \
        mesa-utils \
        libgl1-mesa-dri \
        libgl1 \
        libegl1 \
        libxcb-cursor0 \
        libxcb-xinerama0 \
        libxcb-xkb1 \
        libxkbcommon-x11-0 \
        libsm6 \
        libice6 \
        libfontconfig1 \
        libfreetype6 \
        libdbus-1-3 \
        libglib2.0-0 \
        python3-opengl \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ---------- Qt + Python bindings (SYSTEM) ----------
RUN apt-get update && apt-get install -y \
    python3-pyqt5 \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# ---------- Force matplotlib Qt backend ----------
ENV MPLBACKEND=Qt5Agg

# ---------- pip (NON-GUI ONLY) ----------
RUN pip3 install \
    nanobind \
    dill \
    pandas \
    "numpy<2.0.0" \
    typing_extensions==4.10.0 \
    tyro \
    viser

RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash
RUN apt install -y libeigen-quadprog-dev \
                    libboost-test-dev
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Install HPIPM and BLASFEO
#hpipm install
RUN git clone https://github.com/giaf/blasfeo.git && \
    cd blasfeo && \
    make shared_library -j4 && \
    make install_shared

RUN git clone https://github.com/giaf/hpipm.git && \
    cd hpipm && \
    make shared_library -j4 && \
    make install_shared

RUN echo "/opt/blasfeo/lib" > /etc/ld.so.conf.d/blasfeo.conf && \
    echo "/opt/hpipm/lib" > /etc/ld.so.conf.d/hpipm.conf && \
    ldconfig

RUN cd /hpipm/interfaces/python/hpipm_python && pip3 install .


# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /workspace/ros_ws/src/motion_planner 
WORKDIR /workspace/ros_ws

# Copy package manifests for installing rosdeps
COPY --from=package-manifests /motion_planner/workspace/ros_ws/src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

COPY workspace/ros_ws/src src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DBUILD_TESTING=ON


# Set up entrypoint and working folder.
WORKDIR /workspace/ros_ws
COPY ./.docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc

