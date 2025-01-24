#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

apt-get update
check_success

apt-get install -y python3-pip
check_success

ln -sf $(which pip3) /usr/bin/pip 2>/dev/null || true

echo "deb [trusted=yes arch=amd64] http://deb.repo.autolabor.com.cn $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/autolabor.list
check_success

apt-get update
check_success
apt-get install -y ros-noetic-autolabor
check_success

echo "Installation completed successfully!"
