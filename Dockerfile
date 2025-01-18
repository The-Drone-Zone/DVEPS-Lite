# Use an official Ubuntu as the base image
FROM ubuntu:latest

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cppcheck \
    clang-format \
    cmake \
    g++ \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /workspace

# Set the default command
CMD ["bash"]
