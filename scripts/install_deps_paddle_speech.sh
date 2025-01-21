#!/bin/bash

# Function to check if the previous command succeeded
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error: Previous command failed. Exiting."
        exit 1
    fi
}

# Function to check for NVIDIA GPU
has_gpu() {
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        echo "Detected NVIDIA GPU"
        return 0
    elif [ -c /dev/nvidia0 ]; then
        echo "Detected NVIDIA device"
        return 0
    else
        echo "No NVIDIA GPU detected"
        return 1
    fi
}

# Install PaddlePaddle based on GPU availability
if has_gpu; then
    echo "Installing GPU version of PaddlePaddle..."
    pip install paddlepaddle-gpu==2.5.1 -f https://www.paddlepaddle.org.cn/whl/linux/mkl/avx/stable.html
else
    echo "Installing CPU version of PaddlePaddle..."
    pip install paddlepaddle==2.5.1
fi
check_success

# Verify PaddlePaddle installation
if python3 -c "import paddle; paddle.utils.run_check()"; then
    echo "PaddlePaddle installation verified"
else
    echo "PaddlePaddle installation failed! Check logs for details."
    exit 1
fi

# Install PaddleSpeech
echo "Installing PaddleSpeech..."
pip install paddlespeech==1.4.0
check_success

# Verify installation
if command -v paddlespeech; then
    echo -e "\n\033[32mInstallation successful!\033[0m"
    echo "PaddlePaddle version: $(python3 -c 'import paddle; print(paddle.__version__)')"
    echo "PaddleSpeech version: $(paddlespeech version)"
else
    echo -e "\n\033[31mInstallation failed! Check dependencies.\033[0m"
    exit 1
fi
