---------------------开机启动配置--------------------------------------
1. 文件拷贝
# 将 aoi.service 拷贝到 systemd 服务目录
sudo cp aoi.service /etc/systemd/system/

# 将 start_aoi_simple.sh 拷贝到工作空间根目录，并赋予执行权限
cp start_aoi_simple.sh /home/admin1/dev_ws/
chmod +x /home/admin1/dev_ws/start_aoi_simple.sh

2. 创建 sudo 免密规则
# 创建免密规则（允许 admin1 免密执行 EtherCAT 命令）
echo 'admin1 ALL=(ALL) NOPASSWD: /etc/init.d/ethercat, /usr/bin/ethercat' | sudo tee /etc/sudoers.d/ethercat-aoi
sudo chmod 440 /etc/sudoers.d/ethercat-aoi

# 验证配置
sudo -l -U admin1 | grep ethercat
3. 部署并启用服务
# 拷贝服务文件
sudo cp aoi.service /etc/systemd/system/

# 重载 systemd
sudo systemctl daemon-reload

# 启用开机启动
sudo systemctl enable aoi.service

# 立即启动测试
sudo systemctl start aoi.service

# 查看状态
sudo systemctl status aoi.service

# 查看实时日志
journalctl -u aoi -f
3. 常用管理命令
# 停止服务
sudo systemctl stop aoi.service

# 重启服务
sudo systemctl restart aoi.service

# 禁用开机启动
sudo systemctl disable aoi.service

# 查看历史日志
journalctl -u aoi --since "1 hour ago"

----------------qt界面指令------------------------
# 1. 赋予脚本执行权限
chmod +x ~/dev_ws/launch_qt_hmi.sh

# 2. 拷贝桌面文件到桌面
cp ~/dev_ws/qt_hmi.desktop ~/桌面/

# 3. 右键桌面上的 qt_hmi.desktop → "允许启动"（Allow Launching）

------------------日志管理（logrotate）配置----------------------
1. 创建 logrotate 配置文件
sudo tee /etc/logrotate.d/aoi > /dev/null <<'EOF'
/home/jmin/ros2_launch_logs/aoi.log {
    daily
    rotate 10
    missingok
    notifempty
    copytruncate
    dateext
    dateyesterday
    extension .log
    compress
    delaycompress
}
EOF

2. 配置参数说明
# daily          每天午夜 00:00 无条件切一次（不管大小）
# rotate 10      只保留最近 10 份归档，第 11 份自动删除
# missingok      没有日志文件时不报错
# notifempty     空文件不轮转
# copytruncate   复制后清空原文件，进程不用重启（必须保留）
# dateext        归档名带日期
# dateyesterday  归档名用昨天的日期（与归档内日志时间戳对齐，消除午夜切分的一天偏移）
# extension .log 让 .log 后缀保持在末尾，归档名为 aoi-20260909.log
# compress       压缩旧归档为 .gz
# delaycompress  延迟一个周期再压缩（最新一份归档先不压）
# 说明：若要"满 200M 才切"而不是每天切，把 daily 换成 size 200M 即可

3. 验证
sudo logrotate -d /etc/logrotate.d/aoi   # 调试模式，只模拟不落盘
sudo logrotate -f /etc/logrotate.d/aoi   # 强制切一次实测
ls -lh /home/jmin/ros2_launch_logs/      # 查看归档结果

4. 自动执行说明
# logrotate 靠系统定时任务每天跑一次，本机为 systemd 的 logrotate.timer，
# 每天午夜 00:00 触发（OnCalendar=daily）
# 查看定时器：systemctl list-timers | grep -i logrotate
# 注意：若 /etc/cron.daily/logrotate 也存在会重复跑，可执行
#       sudo rm /etc/cron.daily/logrotate  只保留 timer 一个源

5. 需要改的地方（按需修改）
# 日志目录/文件名：/home/jmin/ros2_launch_logs/aoi.log 按实际环境替换
# 用户名与家目录：jmin、/home/jmin 按实际用户替换
# 切分方式：daily（每天切）/ size 200M（满 200M 切）
# 保留份数：rotate 后的数字
