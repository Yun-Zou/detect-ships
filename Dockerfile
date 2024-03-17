# Download base image Ubuntu 22.04
FROM ubuntu:22.04

# Update Ubuntu Software repository
RUN apt update
RUN apt upgrade -y

# Install minimal prerequisites
RUN apt install -y cmake g++ libopencv-dev