FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i \
    's|http://archive.ubuntu.com/ubuntu|http://de.archive.ubuntu.com/ubuntu|g; \
     s|http://security.ubuntu.com/ubuntu|http://de.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    ninja-build \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    libopenblas-dev \
    libomp-dev \
    net-tools \
    libgl1 \
    libglib2.0-0 \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/\bauto __raw = __to_address(__r.get());/auto __raw = std::__to_address(__r.get());/' \
/usr/include/c++/11/bits/shared_ptr_base.h

RUN python3 -m pip install --upgrade pip setuptools wheel

WORKDIR /workspace
