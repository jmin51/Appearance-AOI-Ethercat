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
