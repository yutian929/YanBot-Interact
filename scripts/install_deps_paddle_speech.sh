#!/bin/bash

# Configuration
IMAGE_REPO="paddlecloud/paddlespeech"
GPU_IMAGE_TAG="develop-gpu-cuda11.2-cudnn8-fb4d25"  # CUDA 11.2 image
CPU_IMAGE_TAG="develop-cpu-fb4d25"                  # CPU image

# Detect GPU
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "[INFO] Detected NVIDIA GPU, using GPU image"
    IMAGE_TAG=$GPU_IMAGE_TAG
    GPU_FLAG="--gpus all"
else
    echo "[INFO] No GPU detected, using CPU image"
    IMAGE_TAG=$CPU_IMAGE_TAG
    GPU_FLAG=""
fi

# Auto-pull image
echo -e "\n[STEP 1/2] Pulling Docker image: ${IMAGE_REPO}:${IMAGE_TAG}"
sudo docker pull ${IMAGE_REPO}:${IMAGE_TAG} || { echo "[ERROR] Pull failed"; exit 1; }

# Generate run command
RUN_CMD="docker run -it --rm --name paddlespeech_ros --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -v \$(pwd):/workspace -e ROS_MASTER_URI=http://${HOST_IP}:11311 -e ROS_HOSTNAME=\$(hostname -I | awk '{print \$1}') --device /dev/snd ${GPU_FLAG} ${IMAGE_REPO}:${IMAGE_TAG} /bin/bash "

# Print instructions
echo -e "\n[STEP 2/2] Run the container with:"
echo -e "\n\033[32m${RUN_CMD}\033[0m"
echo -e "\nAfter running, access the container with:"
echo "   docker exec -it paddlespeech_ros /bin/bash"
