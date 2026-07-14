#!/bin/bash

# 获取节点路径
NODE_PATH=$(find install -name ethercat_node -type f)

# 设置能力
sudo setcap cap_sys_nice,cap_ipc_lock+ep $NODE_PATH

# 验证设置
getcap $NODE_PATH