#!/bin/bash
# ============================================================================
# start_aoi_simple.sh - AOI System Startup Script
# ============================================================================
# 用于 systemd 开机启动，依赖 sudo 免密配置：
#   admin1 ALL=(ALL) NOPASSWD: /etc/init.d/ethercat, /usr/bin/ethercat
# ============================================================================

set -e

# ----------------------------------------------------------------------------
# 日志配置
# ----------------------------------------------------------------------------
LOG_DIR="$HOME/ros2_launch_logs"
mkdir -p "$LOG_DIR"

LOG_FILE="$LOG_DIR/aoi_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "========================================"
echo "启动时间: $(date)"
echo "用户: $(whoami)"
echo "========================================"

# ----------------------------------------------------------------------------
# 辅助函数
# ----------------------------------------------------------------------------
print_step() { echo "[STEP] $1"; }
print_info() { echo "[INFO] $1"; }
print_warn() { echo "[WARN] $1"; }
print_error() { echo "[ERROR] $1"; }

# ----------------------------------------------------------------------------
# EtherCAT 主站管理
# ----------------------------------------------------------------------------
check_ethercat_status() {
    if ! command -v ethercat &> /dev/null; then
        print_error "ethercat 命令未找到"
        return 1
    fi
    sudo ethercat slaves &> /dev/null
}

start_ethercat_master() {
    print_step "启动 EtherCAT 主站"

    if [ ! -f /etc/init.d/ethercat ]; then
        print_error "EtherCAT 服务脚本不存在: /etc/init.d/ethercat"
        return 1
    fi

    # 若已运行则跳过
    if check_ethercat_status; then
        print_info "EtherCAT 主站已在运行，跳过启动"
        return 0
    fi

    # 启动主站
    sudo /etc/init.d/ethercat stop 2>/dev/null || true
    sleep 1

    if sudo /etc/init.d/ethercat start; then
        sleep 3
        if check_ethercat_status; then
            print_info "EtherCAT 主站启动成功"
            sudo ethercat slaves | head -5
            return 0
        fi
    fi

    print_error "EtherCAT 主站启动失败"
    return 1
}

# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
print_step "AOI 系统启动"

# 启动 EtherCAT 主站
start_ethercat_master || exit 1

# 检查 ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
else
    print_warn "未找到 ROS2 环境脚本"
fi

# 检查工作空间
WS_SETUP="$HOME/dev_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    print_warn "工作空间 setup.bash 不存在: $WS_SETUP"
fi

# 启动 ROS2 Launch
print_step "启动 ROS2 系统"
ros2 launch business_logic_py AOI_system.launch.py "$@" &
ROS_PID=$!

print_info "ROS2 PID: $ROS_PID"
wait $ROS_PID

# ----------------------------------------------------------------------------
# 结束
# ----------------------------------------------------------------------------
echo "========================================"
echo "结束时间: $(date)"
echo "日志: $LOG_FILE"
echo "========================================"