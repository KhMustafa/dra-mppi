# catkin_ws/.devcontainer/Dockerfile
FROM osrf/ros:noetic-desktop-full

ARG USERNAME=amr
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip git wget

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-wstool \
    python3-tk \
    python3-catkin-tools \
    python3.8-venv \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (replaces poetry.toml)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
    pytest==7.4.3 \
    pylint==2.17.7 \
    hydra-core==1.3.2 \
    torch==2.4.1 \
    zerorpc==0.6.3 \
    black==23.12.0 \
    numba==0.58.1 \
    pygeos==0.14 \
    ghalton==0.6.1 \
    scipy==1.10.1

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
ENV SHELL /bin/bash

RUN apt-get update && apt-get install -y ros-noetic-derived-object-msgs ros-noetic-robot-localization

# Separate Python 3.11 env for the MPPI planner: newer GPUs (e.g. Blackwell, sm_120)
# need torch built for CUDA 12.8, which has no Python 3.8 wheels. ROS python libs
# (rospy, generated msgs) are pure Python and work from this interpreter.
# The planner launch files pick it up via the MPPI_PYTHON env var.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin INSTALLER_NO_MODIFY_PATH=1 sh \
    && uv venv --python 3.11 /opt/mppi_venv \
    && uv pip install --python /opt/mppi_venv/bin/python \
       --index-url https://download.pytorch.org/whl/cu128 torch==2.11.0 \
    && uv pip install --python /opt/mppi_venv/bin/python \
       "numpy<2" scipy matplotlib seaborn pandas pyyaml rospkg catkin_pkg netifaces \
       hydra-core==1.3.2 ghalton==0.6.1 \
    && chown -R $USERNAME:$USERNAME /opt/mppi_venv
ENV MPPI_PYTHON=/opt/mppi_venv/bin/python

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Set the working directory
WORKDIR /workspace
USER $USERNAME
CMD ["/bin/bash"]
