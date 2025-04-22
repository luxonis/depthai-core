FROM ubuntu:24.04

# Avoid prompts from apt during installation
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    sudo \
    ca-certificates \

    # Library dependanices
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    libspdlog-dev \
    zlib1g-dev \
    libbz2-dev \
    liblzma-dev \
    libfmt-dev \
    libarchive-dev
