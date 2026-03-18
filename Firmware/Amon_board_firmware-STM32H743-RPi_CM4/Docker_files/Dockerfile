# Base Ubuntu image
FROM ubuntu:22.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install common development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    python3 \
    python3-pip \
    git \
    vim \
    curl \
    wget \
    unzip \
    udev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# This will match the volume mount in docker-compose
#WORKDIR /zephyr

# Default shell
#CMD [ "bash" ]
