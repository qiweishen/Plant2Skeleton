ARG UBUNTU_TAG=22.04
FROM docker.io/ubuntu:${UBUNTU_TAG}

# Prevent interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# ============================================================
# Base Development Tools
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    # ----- Version Control -----
    git \
    # ----- Debugging & Analysis -----
    gdb \
    lldb \
    valgrind \
    # ----- Code Formatting -----
    clang-format \
    # Utilities
    wget \
    curl \
    ca-certificates \
    sudo \
    locales \
    # Core libraries
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    # Python
    python3-dev \
    python3-pip

# ============================================================
# Locale Configuration
# ============================================================
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================
# Easy3D Dependencies
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # OpenGL / Graphics
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglfw3-dev \
    libglew-dev \
    # X11 (windowing and input)
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libxext-dev \
    libx11-dev \
    # CGAL (computational geometry)
    libcgal-dev \
    # Qt6 (GUI framework)
    qt6-base-dev \
    qt6-tools-dev \
    libqt6opengl6-dev \
    # FFmpeg (video encoding)
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Build and Install Easy3D
# ============================================================
ARG Easy3D_GIT_TAG=v2.6.1
ARG Easy3D_ENABLE_CGAL=ON
ARG Easy3D_ENABLE_QT=ON
ARG Easy3D_ENABLE_FFMPEG=ON

RUN mkdir -p /src && cd /src && \
    git clone https://github.com/LiangliangNan/Easy3D.git --branch ${Easy3D_GIT_TAG} && \
    cd Easy3D && \
    mv README.md ReadMe.md && \
    mkdir Release && cd Release && \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DEasy3D_ENABLE_CGAL=${Easy3D_ENABLE_CGAL} \
        -DEasy3D_ENABLE_QT=${Easy3D_ENABLE_QT} \
        -DEasy3D_ENABLE_FFMPEG=${Easy3D_ENABLE_FFMPEG} \
        .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# ============================================================
# Create Non-root User
# ============================================================
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    # Grant passwordless sudo
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ============================================================
# Workspace Setup
# ============================================================
WORKDIR /workspace
RUN chown ${USERNAME}:${USERNAME} /workspace

USER ${USERNAME}
