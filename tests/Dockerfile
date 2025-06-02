FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV DISPLAY=:99


# Install ca-certificates first to fix HTTPS errors
RUN apt-get update && apt-get install -y ca-certificates && update-ca-certificates

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        pkg-config \
        cmake \
        git \
        python3 \
        python3-pip \
        python3-venv \
        libopencv-dev \
        xvfb \
        x11-utils \
        locales \
        wget \
        curl

RUN sleep 3

RUN apt-get update && \
        apt-get install -y \
        zip \
        unzip \
        tar \
        nasm \
        bison \
        libx11-dev \
        libxft-dev \
        libxext-dev \
        autoconf
        
RUN sleep 5
RUN apt-get update && \
        apt-get install -y \
        automake \
        libtool \
        make \
        libusb-1.0 \
        ninja-build \
        python3-dev \
        libxrandr-dev \
        libxdamage-dev \
        libxi-dev \
        libxtst-dev \
        libgles2-mesa-dev \
        libxinerama-dev && \
    rm -rf /var/lib/apt/lists/*


RUN locale-gen en_US.UTF-8

# Set workdir to your project directory
WORKDIR /workspace

# Copy project files
COPY . .

# Create venv and install Python dependencies
RUN python3 -m venv venv && \
    . venv/bin/activate && \
    pip install --upgrade pip && \
    pip install numpy pytest pytest-html

# Build the C++ project (no tests run yet)
RUN cmake -S . -B build \
    -D CMAKE_BUILD_TYPE=Release \
    -D DEPTHAI_BUILD_EXAMPLES=ON \
    -D DEPTHAI_VCPKG_INTERNAL_ONLY=OFF \
    -D DEPTHAI_BUILD_TESTS=ON \
    -D DEPTHAI_TEST_EXAMPLES=ON \
    -D DEPTHAI_BUILD_PYTHON=ON \
    -D DEPTHAI_PYTHON_TEST_EXAMPLES=ON \
    -D DEPTHAI_PYTHON_ENABLE_EXAMPLES=ON && \
    cmake --build build --parallel 8 --config Release

# Set entrypoint for runtime test execution
COPY run_tests_entrypoint.sh /run_tests_entrypoint.sh
RUN chmod +x /run_tests_entrypoint.sh

ENTRYPOINT ["/run_tests_entrypoint.sh"]
