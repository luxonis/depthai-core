FROM debian:trixie-slim AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        zip \
        unzip \
        libopencv-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        ninja-build \
        pkg-config \
        nasm \
        python3 \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        patchelf && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages auditwheel

WORKDIR /opt/depthai-core
COPY . .

RUN if [ -d .git ]; then git submodule update --init --recursive; fi

RUN rm -rf build && \
    cmake -S . -B build \
        -G Ninja \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DDEPTHAI_BUILD_EXAMPLES=OFF && \
    cmake --build build && \
    cmake --install build

WORKDIR /opt/depthai-core/bindings/python
RUN python3 -m pip wheel . -w /opt/wheels

# Repair wheel (to copy all the dependencies inside the wheel)
RUN mkdir -p /opt/wheels_repaired && \
    auditwheel repair /opt/wheels/*.whl -w /opt/wheels_repaired


RUN python3 -m pip install --break-system-packages /opt/wheels_repaired/*.whl


FROM debian:trixie-slim AS deployment

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        zip \
        unzip \
        libopencv-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        ninja-build \
        pkg-config \
        nasm \
        python3 \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages opencv-python-headless

# ------------------------------------------------------------
# Copy final installed artifacts only
# ------------------------------------------------------------
COPY --from=builder /usr/local /usr/local

# -------------------------
# C++ sanity check
# -------------------------
WORKDIR /opt/depthai-test
RUN cat <<'EOF' > main.cpp
#include <depthai/depthai.hpp>
#include <depthai/build/version.hpp>
#include <iostream>

int main() {
    std::cout << "DepthAI commit: " << dai::build::COMMIT << std::endl;
    return 0;
}
EOF

RUN cat <<'EOF' > CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(depthai_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(depthai CONFIG REQUIRED)

add_executable(depthai_test main.cpp)
target_link_libraries(depthai_test PRIVATE depthai::core)
EOF

RUN cmake -S . -B build -G Ninja && \
    cmake --build build && \
    ./build/depthai_test

# -------------------------
# Python sanity check
# -------------------------
RUN python3 - <<'EOF'
import depthai as dai
print("DepthAI version:", dai.__version__)
print("DepthAI module:", dai.__file__)
EOF
