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

# Auto-pull the image
echo -e "\n[STEP 1/2] Pulling Docker image..."
sudo docker pull ${IMAGE_REPO}:${IMAGE_TAG}

if [ $? -ne 0 ]; then
    echo -e "\n[ERROR] Failed to pull the image. Check your network connection or image tag."
    exit 1
fi

# Generate run command
RUN_CMD="docker run -d --name paddlespeech \\
    ${GPU_FLAG} \\
    -v \$(pwd):/workspace \\
    -p 8888:8888 \\
    ${IMAGE_REPO}:${IMAGE_TAG} \\
    jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --notebook-dir=/workspace"

# Print next steps
echo -e "\n[STEP 2/2] Image pulled successfully! Run the container with:"
echo -e "\n\033[32m${RUN_CMD}\033[0m"
echo -e "\nAfter running the container:"
echo "1. Get Jupyter token:"
echo "   docker exec paddlespeech jupyter server list"
echo "2. Access Jupyter Lab:"
echo "   http://localhost:8888/lab"
echo "3. Verify GPU support (for GPU image):"
echo "   docker exec paddlespeech python -c \"import paddle; print(paddle.is_compiled_with_cuda())\""
