#!/usr/bin/env bash

SESSION=a2_sim
SLEEP_TIME=3

# 停止会话函数
stop_session() {
    echo ""
    echo "Stopping session $SESSION..."
    
    # 向每个窗口发送 Ctrl+C 停止程序
    for window in 0 1 2; do
        tmux send-keys -t $SESSION:$window C-c 2>/dev/null
        sleep 0.5
    done
    
    sleep 1
    
    # 杀死会话
    tmux kill-session -t $SESSION 2>/dev/null
    
    echo "Session $SESSION stopped."
    exit 0
}

# 处理 Ctrl+C 信号
trap stop_session SIGINT SIGTERM

# 检查是否传入 stop 参数
if [ "$1" == "stop" ]; then
    stop_session
fi

# 如果 session 已存在，直接退出
tmux has-session -t $SESSION 2>/dev/null
if [ $? == 0 ]; then
    echo "Session $SESSION already exists."
    echo "Run '$0 stop' to stop it or 'tmux attach -t $SESSION' to attach."
    exit 1
fi

# 创建 session
tmux new-session -d -s $SESSION
sleep $SLEEP_TIME

# ---------- Window 1: mujoco sim ----------
tmux rename-window -t $SESSION:0 mujoco
tmux send-keys -t $SESSION:0 \
"docker start a2_deploy && docker exec -it a2_deploy /bin/bash" C-m
sleep $SLEEP_TIME

tmux send-keys -t $SESSION:0 \
"cd /home/agi/a2_deploy_workspace/mujoco_sim_install/bin" C-m

tmux send-keys -t $SESSION:0 \
"./start_a2_t2d0_ultra.sh" C-m
sleep $SLEEP_TIME

# ---------- Window 2: RL control sim ----------
tmux new-window -t $SESSION -n rl_control
sleep $SLEEP_TIME

tmux send-keys -t $SESSION:1 \
"docker start a2_deploy && docker exec -it a2_deploy /bin/bash" C-m
sleep $SLEEP_TIME

tmux send-keys -t $SESSION:1 \
"cd /home/agi/a2_deploy_workspace/deploy" C-m

tmux send-keys -t $SESSION:1 \
"bash install/deploy_assets/scripts/start_rl_control_sim.sh" C-m
sleep $SLEEP_TIME

# ---------- Window 3: joy interface ----------
tmux new-window -t $SESSION -n joy
sleep $SLEEP_TIME

tmux send-keys -t $SESSION:2 \
"docker start a2_deploy && docker exec -it a2_deploy /bin/bash" C-m
sleep $SLEEP_TIME

tmux send-keys -t $SESSION:2 \
"cd /home/agi/a2_deploy_workspace/deploy" C-m

tmux send-keys -t $SESSION:2 \
"python3 install/deploy_assets/scripts/joy_interface.py" C-m

echo "========================================="
echo "Session $SESSION created successfully!"
echo "========================================="
echo ""
echo "Session is running in background."
echo ""
echo "Available commands:"
echo "  $0 stop           - Stop the session"
echo "  tmux attach -t $SESSION - Attach to the session"
echo "  tmux ls           - List all sessions"
echo ""
echo "Press Ctrl+C in this terminal to stop the session"
echo ""

# 保持脚本运行，等待信号
echo "Script is running. Waiting for Ctrl+C signal..."
echo ""

# 使用无限循环保持脚本运行，等待信号
while true; do
    # 检查会话是否还存在（如果意外结束）
    tmux has-session -t $SESSION 2>/dev/null
    if [ $? != 0 ]; then
        echo "Warning: Session $SESSION has ended unexpectedly."
        echo "Exiting script..."
        exit 1
    fi
    
    # 睡眠一小段时间，避免占用太多CPU
    sleep 2
    
    # 可以在这里添加其他检查或状态显示
    # 例如：显示会话运行时间等
done