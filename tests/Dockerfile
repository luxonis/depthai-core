FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV DISPLAY=:99
ARG FLAVOR=vanilla
ENV FLAVOR=${FLAVOR}
ARG PARALLEL_JOBS=8
ARG PULL_REQUEST="false"

# Update and install all required packages in one go
RUN apt-get update && \
    apt-get install -y \
        ca-certificates \
        build-essential \
        pkg-config \
        cmake \
        git \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        libopencv-dev \
        xvfb \
        x11-utils \
        locales 
RUN sleep 5
RUN apt-get update && \
        apt-get install -y \
        wget \
        curl \
        unzip \
        zip \
        tar \
        nasm \
        bison \
        libx11-dev \
        libxft-dev \
        libxext-dev \
        autoconf \
        automake \
        libtool
RUN sleep 5
RUN apt-get update && \
        apt-get install -y \
        make \
        libusb-1.0 \
        libxrandr-dev \
        libxdamage-dev \
        libxi-dev \
        libxtst-dev \
        libgles2-mesa-dev \
        meson
RUN sleep 5
RUN apt-get update && \
        apt-get install -y \
        libxinerama-dev \
        libgl1-mesa-dev \
        libegl1 \
        libxcursor-dev \
        libxkbcommon-x11-0 \
        libudev-dev \
        g++ \
        ninja-build && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# Set workdir to your project directory
WORKDIR /workspace

# Copy project files
ARG BRANCH=v3_develop
ARG REPO_URL=https://github.com/luxonis/depthai-core.git
ARG GIT_COMMIT=unspecified  # commit SHA busts cache

RUN echo "Cloning depthai-core at commit ${GIT_COMMIT}"

RUN if [ "$PULL_REQUEST" = "false" ]; then \
        git clone --recursive -b ${BRANCH} ${REPO_URL} . ; \
    else \
        git clone ${REPO_URL} . &&\
        git fetch --depth 1 origin ${BRANCH}:pr-merge && \
        git switch pr-merge && \
        git submodule update --init --recursive; \
    fi

# Create venv and install Python dependencies
RUN python3 -m venv venv && \
    . venv/bin/activate && \
    pip install --upgrade pip && \
    pip install numpy pytest pytest-html

ENV PATH="/workspace/venv/bin:$PATH"

# Set toolchain path only if flavor != vanilla
RUN if [ "$FLAVOR" != "vanilla" ]; then \
        export CMAKE_TOOLCHAIN_PATH=/workspace/cmake/toolchain/${FLAVOR}.cmake && \
        echo "Using toolchain: $CMAKE_TOOLCHAIN_PATH"; \
    else \
        echo "Using vanilla flavor (no toolchain)"; \
    fi && \
    cmake -S . -B build \
        -D CMAKE_BUILD_TYPE=Release \
        -D DEPTHAI_VCPKG_INTERNAL_ONLY=OFF \
        -D VCPKG_OVERLAY_TRIPLETS="/workspace/cmake/triplets/release" \
        -D DEPTHAI_BUILD_TESTS=ON \
        $(if [ "$PULL_REQUEST" = "false" ]; then echo "-D DEPTHAI_BUILD_EXAMPLES=ON \
        -D DEPTHAI_TEST_EXAMPLES=ON \
        -D DEPTHAI_BUILD_PYTHON=ON \
        -D DEPTHAI_PYTHON_TEST_EXAMPLES=ON \
        -D DEPTHAI_PYTHON_ENABLE_EXAMPLES=ON"; fi)


RUN cmake --build build --parallel $PARALLEL_JOBS --config Release
