#!/bin/bash
cd "$(dirname "$0")"

# Start the Docker image build
rm -rf ./tmp_sources
mkdir -p ./tmp_sources

# Ensure source files exist before copying
if [ -d "../src" ]; then
  cp -r ../src/* ./tmp_sources
else
  echo "Error: Source directory ../src does not exist. Exiting."
  exit 1
fi

if docker build --network=host -t armando_simulation_image -f ./Dockerfile .; then
  rm -rf ./tmp_sources
  echo "Build completed. Start the container with: ./docker_run_container.sh"
else
  rm -rf ./tmp_sources
  echo "Rerun ./docker_build_image.sh and try again."
fi
