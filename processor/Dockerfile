# Use an official Ubuntu as the base image
FROM ubuntu:latest

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cppcheck \
    clang-format \
    cmake \
    g++ \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Download and install Google Test
RUN git clone https://github.com/google/googletest.git /googletest && \
    mkdir -p /googletest/build && \
    cd /googletest/build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf /googletest

# Set up the working directory
WORKDIR /workspace

# Copy project files into the container
COPY . /workspace

# Set the default command
CMD ["bash"]
