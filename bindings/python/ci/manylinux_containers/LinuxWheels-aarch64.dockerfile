FROM quay.io/opencv-ci/opencv-python-manylinux2014-aarch64:20231225
USER root

# Clone the OpenCV repository
RUN git clone https://github.com/opencv/opencv.git /tmp/opencv

# Setup the build environment
RUN cd /tmp/opencv && git checkout 4.9.0
RUN cd /tmp/opencv && cmake -S . -B build -D CMAKE_BUILD_TYPE=RELEASE

# Buiild the OpenCV library
RUN cd /tmp/opencv && cmake --build build --target install --parallel 8