#!/bin/bash

# Set the Docker image name
IMAGE_NAME="foxy-desktop"

# Build the Docker image
docker build -t $IMAGE_NAME .
