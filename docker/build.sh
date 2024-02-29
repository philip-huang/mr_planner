#!/bin/bash

# Build the Docker image
docker build -t mr_planner .

# Check if the build was successful
if [ $? -eq 0 ]; then
  echo "Docker image built successfully as mr_planner"
else
  echo "Failed to build Docker image"
fi
