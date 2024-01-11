#!/usr/bin/env bash

ICD_PATH_1="/usr/share/vulkan/icd.d/nvidia_icd.json"
ICD_PATH_2="/etc/vulkan/icd.d/nvidia_icd.json"
LAYERS_PATH_1="/usr/share/vulkan/icd.d/nvidia_layers.json"
LAYERS_PATH_2="/usr/share/vulkan/implicit_layer.d/nvidia_layers.json"
LAYERS_PATH_3="/etc/vulkan/implicit_layer.d/nvidia_layers.json"
EGL_VENDOR_PATH="/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
DEFAULT_DATA_DIR="$SCRIPT_DIR/omnigibson_data"
DATA_PATH=$DEFAULT_DATA_DIR


# Find the layers file
if [ -e "$LAYERS_PATH_1" ]; then
    LAYERS_PATH=$LAYERS_PATH_1
elif [ -e "$LAYERS_PATH_2" ]; then
    LAYERS_PATH=$LAYERS_PATH_2
elif [ -e "$LAYERS_PATH_3" ]; then
    LAYERS_PATH=$LAYERS_PATH_3
else
    echo "Missing nvidia_layers.json file."
    echo "Typical paths:";
    echo "- /usr/share/vulkan/icd.d/nvidia_layers.json";
    echo "- /usr/share/vulkan/implicit_layer.d/nvidia_layers.json";
    echo "- /etc/vulkan/implicit_layer.d/nvidia_layers.json";
    echo "You can google nvidia_layers.json for your distro to find the correct path.";
    echo "Consider updating your driver to 525 if you cannot find the file.";
    echo "To continue update the LAYERS_PATH_1 at the top of the run_docker.sh file and retry";
    exit;
fi 

# Find the ICD file
if [ -e "$ICD_PATH_1" ]; then
    ICD_PATH=$ICD_PATH_1
elif [ -e "$ICD_PATH_2" ]; then
    ICD_PATH=$ICD_PATH_2
else
    echo "Missing nvidia_icd.json file.";
    echo "Typical paths:";
    echo "- /usr/share/vulkan/icd.d/nvidia_icd.json or";
    echo "- /etc/vulkan/icd.d/nvidia_icd.json";
    echo "You can google nvidia_icd.json for your distro to find the correct path.";
    echo "Consider updating your driver to 525 if you cannot find the file.";
    echo "To continue update the ICD_PATH_1 at the top of the run_docker.sh file and retry";
    exit;
fi

xhost +local:root
OMNIGIBSON_HEADLESS=0
DOCKER_DISPLAY=$DISPLAY


sudo docker rm -f omnigibson_env

sudo docker run -it --name omnigibson_env --network host\
    --gpus all \
    --privileged \
    -e DISPLAY=${DOCKER_DISPLAY} \
    -e OMNIGIBSON_HEADLESS=${OMNIGIBSON_HEADLESS} \
    -v ${ICD_PATH}:/etc/vulkan/icd.d/nvidia_icd.json \
    -v ${LAYERS_PATH}:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
    -v ${EGL_VENDOR_PATH}:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    -v $DATA_PATH/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
    -v $DATA_PATH/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v $DATA_PATH/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v $DATA_PATH/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v $DATA_PATH/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v $DATA_PATH/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v $DATA_PATH/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
    -v $DATA_PATH/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v $DATA_PATH/isaac-sim/documents:/root/Documents:rw \
    -v $HOME/Desktop/shared:/shared \
    jieyitsinghuawx/icra2024-sim2real-axs-env:v0.0.3 \
    /bin/bash
