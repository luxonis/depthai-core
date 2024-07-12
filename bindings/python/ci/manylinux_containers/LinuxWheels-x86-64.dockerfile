FROM quay.io/opencv-ci/opencv-python-manylinux2014-x86-64:20231225
USER root
WORKDIR /work

RUN sed -i 's/enabled=1/enabled=0/g' /etc/yum/pluginconf.d/fastestmirror.conf && \
    sed -i 's/^mirrorlist/#mirrorlist/g' /etc/yum.repos.d/*.repo && \
    sed -i 's;^.*baseurl=http://mirror;baseurl=https://vault;g' /etc/yum.repos.d/*.repo && \
    yum install epel-release -y


# lz4 for flann, mpi for boost-mpi, libatomic for pcl
RUN yum install -y lz4-devel \
    openmpi \
    openmpi-devel \
    libatomic \
    devtoolset-10-libatomic-devel

# Clone the OpenCV repository
RUN git clone https://github.com/opencv/opencv.git /tmp/opencv

# Setup the build environment
RUN cd /tmp/opencv && git checkout 4.9.0
RUN cd /tmp/opencv && cmake -S . -B build -D CMAKE_BUILD_TYPE=RELEASE

# Buiild the OpenCV library
RUN cd /tmp/opencv && cmake --build build --target install --parallel 8

# Update the package list and install necessary packages
RUN yum -y update && \
    yum install -y \
        sudo \
        git \
        wget

# Install PCL
# lz4 for flann, mpi for boost-mpi, libatomic for pcl
RUN yum install -y lz4-devel \
    openmpi \
    openmpi-devel \
    libatomic \
    devtoolset-10-libatomic-devel

RUN git clone --branch 1.9.2 https://github.com/flann-lib/flann.git \
    && cd flann \
    && cmake -S $(pwd) -Bbuild \
    -D'CMAKE_BUILD_TYPE=Release' \
    -D'BUILD_MATLAB_BINDINGS=OFF' \
    -D'BUILD_EXAMPLES=OFF' \
    -D'BUILD_TESTS=OFF' \
    -D'CMAKE_CXX_FLAGS=-fopenmp' \
    -D'CMAKE_C_FLAGS=-fopenmp' \
    -D'BUILD_DOC=OFF' \
    && cmake --build build --target install --parallel 8

RUN wget http://downloads.sourceforge.net/project/boost/boost/1.83.0/boost_1_83_0.tar.gz \
    && tar xfz boost_1_83_0.tar.gz \
    && rm boost_1_83_0.tar.gz \
    && cd boost_1_83_0 \
    && . /etc/profile.d/modules.sh \
    && module load mpi \
    && ./bootstrap.sh --prefix=/usr/local --with-libraries=system,filesystem,mpi,iostreams,serialization \
    && sed -i '1s/^/using mpi ;\n/' project-config.jam \
    && ./b2 variant=release install

RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && tar xfz eigen-3.4.0.tar.gz \
    && rm eigen-3.4.0.tar.gz \
    && cd eigen-3.4.0 \
    && cmake -S. -Bbuild -D'CMAKE_BUILD_TYPE=Release' \
    && cmake --build build --target install

RUN git clone --branch pcl-1.14.1 https://github.com/PointCloudLibrary/pcl.git

RUN cd pcl \
    && cmake -S. -Bbuild \
    -D'WITH_VTK=OFF' \
    -D'WITH_PNG=OFF' \
    -D'PCL_SHARED_LIBS=ON' \
    -D'PCL_ENABLE_SSE=OFF' \
    -D'BUILD_TESTS=OFF' \
    -D'BUILD_examples=OFF'\
    -D'WITH_OPENGL=OFF' \
    -D'CMAKE_C_STANDARD=99' \
    -D'CMAKE_POSITION_INDEPENDENT_CODE=ON' \
    -D'CMAKE_CXX_FLAGS=-fopenmp -O3 -fno-omit-frame-pointer -mno-omit-leaf-frame-pointer -fstack-protector-strong -fstack-clash-protection -Wformat -Werror=format-security' \
    -D'CMAKE_C_FLAGS=-fopenmp' \
    -D'CMAKE_BUILD_TYPE=Release' \
    -D'PCL_INDEX_SIGNED=true' \
    -D'PCL_INDEX_SIZE=32' \
    -D'PCL_ONLY_CORE_POINT_TYPES=ON' \
    && cmake --build build --target install --parallel 8
RUN yum install -y perl-core
RUN yum erase -y git