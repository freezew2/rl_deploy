#!/bin/bash

set -e pipefail

source /opt/ros/humble/setup.bash

SHELL_FOLDER=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $SHELL_FOLDER
pushd "$SHELL_FOLDER"/../../ || exit
    echo $pwd
    source setup.bash
    export LD_LIBRARY_PATH=./deploy_assets/thirdparty/onnxruntime-linux-x64-1.19.2/lib:$LD_LIBRARY_PATH
    export PLUGIN_INSTALL_DIR=/home/agi/a2_deploy_workspace/deploy/install/legged_system/bin
    export AIMRT_CFG_PATH=../install/legged_system/bin/ros_chn_benchmark_cfg.yaml
   if [ -f /dev/shm/iox1_0_u_agiuser ]; then
        ln -f /dev/shm/iox1_0_u_agiuser /dev/shm/iox1_0_u_agi
        echo "Symlink created: /dev/shm/iox1_0_u_agi -> /dev/shm/iox1_0_u_agiuser"
    else
        echo "Warning: Host shared memory file 'iox1_0_u_agiuser' not found!"
    fi
    USER=agiuser ros2 launch rl_controllers rl_control_real.launch.py
popd || exit
