
sudo /etc/init.d/ethercat restart
ethercat slaves
ethercat cstruct -p 0

ethercat download -p 0 0x6040 00 0x0006 --type uint16
ethercat upload -p 0 0x6041 00 --type uint16

colcon build

source install/setup.bash

