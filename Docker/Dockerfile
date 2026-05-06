ARG UBUNTU_TAG=22.04
FROM docker.io/ubuntu:${UBUNTU_TAG}

# Prevent interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# ============================================================
# System packages
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ----- Build essentials ----- \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    # ----- Version Control ----- \
    git \
    # ----- Debugging & Analysis ----- \
    gdb \
    lldb \
    valgrind \
    # ----- File system ----- \
    zenity \
    # ----- Code Formatting ----- \
    clang-format \
    # ----- Utilities ----- \
    wget \
    curl \
    ca-certificates \
    sudo \
    locales \
    # ----- C++ Libraries ----- \
    libomp-dev \
    libboost-all-dev \
    # ----- OpenGL / Graphics ----- \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglfw3-dev \
    libglew-dev \
    # ----- X11 (windowing and input) ----- \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libxext-dev \
    libx11-dev \
    # ----- Qt6 (GUI framework) ----- \
    qt6-base-dev \
    qt6-tools-dev \
    libqt6opengl6-dev \
    # ----- Python ----- \
    python3-dev \
    python3-pip \
    python3-venv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Python
# ============================================================
RUN pip install --no-cache-dir \
    numpy \
    scipy \
    scikit-learn \
    plyfile

# ============================================================
# Locale Configuration
# ============================================================
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================
# Create Non-root User
# UID/GID come from build args so they can match the host user;
# pass them via docker-compose `args:` (USER_UID / USER_GID).
# ============================================================
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    # Grant passwordless sudo \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ============================================================
# Workspace Setup
# ============================================================
WORKDIR /workspace
RUN chown ${USERNAME}:${USERNAME} /workspace

USER ${USERNAME}
