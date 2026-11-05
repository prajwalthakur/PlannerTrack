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
# RUN add-apt-repository ppa:kisak/kisak-mesa && \
#     apt-get update && apt-get install -y \
#         mesa-utils \
#         libgl1-mesa-dri \
#         libgl1 \
#         libegl1 \
#         libxcb-cursor0 \
#         libxcb-xinerama0 \
#         libxcb-xkb1 \
#         libxkbcommon-x11-0 \
#         libsm6 \
#         libice6 \
#         libfontconfig1 \
#         libfreetype6 \
#         libdbus-1-3 \
#         libglib2.0-0 \
#         python3-opengl \
#         ffmpeg \
#     && rm -rf /var/lib/apt/lists/*

# ---------- OpenGL runtime (Noble / 24.04 compatible) ----------
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglx0 \
    libglvnd0 \
    libegl1 \
    libx11-6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# ---------- RViz2 ----------
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-rendering \
    ros-${ROS_DISTRO}-rviz-ogre-vendor \
    && rm -rf /var/lib/apt/lists/*



# RUN apt-get update && apt-get install -y \
#     ros-${ROS_DISTRO}-rviz2 \
#     ros-${ROS_DISTRO}-rviz-common \
#     ros-${ROS_DISTRO}-rviz-ogre-vendor \
#     && rm -rf /var/lib/apt/lists/*


# RUN apt-get update && apt-get install -y \
#     libopengl0 \
#     libglvnd0 \
#     libgl1-mesa-glx \
#     && rm -rf /var/lib/apt/lists/*



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
    viser \
    ruff

# ---------- PyTorch (CUDA build) ----------
# pip CUDA wheels ship their own CUDA runtime (cuDNN/cuBLAS/NCCL), so no
# nvidia/cuda base image or CUDA toolkit is required here. Needs only a recent
# host NVIDIA driver (host = 595.84) + nvidia-container-toolkit, and running the
# container with `--gpus all`.
ARG TORCH_INDEX=https://download.pytorch.org/whl/cu124
RUN pip3 install --index-url ${TORCH_INDEX} torch==2.6.0 \
 && pip3 install "numpy<2.0.0"

# ---------- JAX (CUDA build) ----------
# Same deal as torch above: the `jax[cuda12]` pip extra bundles its own
# cuDNN/cuBLAS/NCCL via nvidia-*-cu12 packages, so it doesn't need (or share)
# a system CUDA toolkit, and it doesn't conflict with torch's copies -- each
# loads its own .so's at import time.
# Pin nvidia-cuda-nvcc-cu12 to 12.4.131 (matches cu124): newer nvcc wheels
# (>=12.5) ship nvidia/cuda_nvcc as a pure namespace package with no
# __init__.py, which breaks jaxlib 0.4.34's _cuda_path() (__file__ is None).
RUN pip3 install "jax[cuda12]==0.4.35" "numpy<2.0.0" "nvidia-cuda-nvcc-cu12==12.4.131"

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && apt-get update && \
    apt-get install -y cuda-toolkit-12-6 && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y libopencv-dev && rm -rf /var/lib/apt/lists/*

RUN curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | bash
RUN apt install -y libeigen-quadprog-dev \
                    libboost-test-dev
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# ---------- planner / controller deps not covered by rosdep ----------
# nlohmann-json: planner_base (mpl_route_planner). nav2-*: mppi_controller's
# trajectory_follower_node and the racing launch. poppler-utils: PDF reading.
RUN apt-get update && apt-get install -y \
    nlohmann-json3-dev \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    poppler-utils \
    && rm -rf /var/lib/apt/lists/*

# ---------- leap-c (differentiable acados MPC, torch backend) ----------
# https://leap-c.github.io/leap-c/installation.html. Installed into the system
# python (no venv) so it shares the interpreter with rclpy and the CUDA
# torch/jax above. The pinned leap-c commit also pins its acados submodule.
# acados is built with the OpenMP flags leap-c requires.
# The constraints file stops pip from replacing torch/jax or the apt-provided
# numpy/scipy/matplotlib while resolving leap-c's unpinned deps (casadi etc.).
# t_renderer is fetched at build time, otherwise acados_template prompts for it
# interactively on the first solver codegen.
ARG LEAP_C_REF=217e1c8ae0d47b4219ee8ebd69630f48e3cf3eff
ENV ACADOS_SOURCE_DIR=/opt/leap-c/external/acados
# HPIPM/BLASFEO come only from acados' submodules (no standalone build), so
# there is a single copy: libs in ${ACADOS_SOURCE_DIR}/lib, headers in
# ${ACADOS_SOURCE_DIR}/include/{hpipm,blasfeo}/include. hpipm_python is
# installed from that same source and finds libhpipm.so via LD_LIBRARY_PATH.
# acados builds hpipm with CMake, which skips hpipm's interfaces/c, so the
# python hpipm_ocp_qp_solver2 / hpipm_ocp_qp_solver_arg2 classes are
# unavailable (missing d_ocp_qp_solver_* symbols); the rest of hpipm_python works.
ENV LD_LIBRARY_PATH=${ACADOS_SOURCE_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
RUN git clone https://github.com/leap-c/leap-c.git /opt/leap-c && \
    cd /opt/leap-c && \
    git checkout ${LEAP_C_REF} && \
    git submodule update --init --recursive && \
    mkdir -p external/acados/build && cd external/acados/build && \
    cmake .. -DACADOS_WITH_OPENMP=ON -DACADOS_NUM_THREADS=1 -DACADOS_PYTHON=ON \
             -DCMAKE_BUILD_TYPE=Release && \
    make install -j$(nproc) && \
    cd /opt/leap-c && \
    printf '%s\n' torch==2.6.0 jax==0.4.35 jaxlib==0.4.34 numpy==1.26.4 \
                  scipy==1.11.4 matplotlib==3.6.3 casadi==3.8.1 > /tmp/leap-c-constraints.txt && \
    pip3 install -c /tmp/leap-c-constraints.txt \
        -e external/acados/interfaces/acados_template \
        -e ".[torch,test]" \
        external/acados/external/hpipm/interfaces/python/hpipm_python && \
    python3 -c "from acados_template.utils import get_tera; get_tera(force_download=True)" && \
    rm /tmp/leap-c-constraints.txt


# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /workspace/ros_ws/src/motion_planner 
WORKDIR /workspace/ros_ws

# Copy package manifests for installing rosdeps
COPY --from=package-manifests /motion_planner/workspace/ros_ws/src ./src

# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

COPY workspace/ros_ws/src src

# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && colcon build --cmake-args -DBUILD_TESTING=ON


# Set up entrypoint and working folder.
WORKDIR /workspace/ros_ws
COPY ./.docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# ---------- non-root dev user (matches host UID/GID so bind-mounted files ----------
# ---------- under workspace/ros_ws/src aren't left root-owned on the host) --------
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=dev
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* && \
    if ! getent passwd ${USERNAME} > /dev/null; then \
      existing_user="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
      [ -n "$existing_user" ] && userdel -r "$existing_user" 2>/dev/null; \
      existing_group="$(getent group ${USER_GID} | cut -d: -f1)"; \
      [ -n "$existing_group" ] && groupdel "$existing_group" 2>/dev/null; \
      groupadd --gid ${USER_GID} ${USERNAME} && \
      useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi && \
    usermod -aG video,dialout ${USERNAME} 2>/dev/null || true && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    chown -R ${USERNAME}:${USERNAME} /workspace

USER ${USERNAME}
ENV HOME=/home/${USERNAME}
RUN echo "source /entrypoint.sh" >> ${HOME}/.bashrc

