#!/bin/bash

# Set the image name
IMAGE_NAME="dvrk"

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME -f ./Dockerfile .

# Check if the image build was successful
if [ $? -ne 0 ]; then
    echo "Failed to build the Docker image."
    exit 1
fi
