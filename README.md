# 動作方法
```shell
cd /etc/netplan

sudo mv 00-original.yaml 02-original.yaml

sudo netplan try

sudo systemctl start hostapd

sudo systemctl restart hostapd
```

```shell
cd ./earth/ros2_ws

colcon build

source ./install/local_setup.zsh

ros2 run micro_ros_agent micro_ros_agent udp4 --port 2000
```

```shell
cd ./earth/ros2_ws

source ./install/local_setup.zsh

ros2 launch viewer viewer.launch.py
```
