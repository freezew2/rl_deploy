#!/bin/bash

set -e pipefail

source /opt/ros/humble/setup.bash

SHELL_FOLDER=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

pushd "$SHELL_FOLDER"/../../ > /dev/null

pwd

source setup.bash

export LD_LIBRARY_PATH="$PWD/deploy_assets/thirdparty/onnxruntime-linux-x64-1.19.2/lib:$LD_LIBRARY_PATH"
export PLUGIN_INSTALL_DIR="$PWD/legged_system/bin"
export AIMRT_CFG_PATH="$PWD/legged_system/bin/ros2_chn_cfg.yaml"

ros2 launch rl_controllers rl_control_real.launch.py

popd > /dev/null
