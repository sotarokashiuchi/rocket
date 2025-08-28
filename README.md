# ロケットプロジェクト
和歌山高専の学生と和歌山大学の学生と共同でロケットを打ち上げるプロジェクト  
このリポジトリには電装部に関する回路図、ソースコード等がまとめられています

## 構成図
![](./_attachments/fig.drawio.svg)

## 動作方法
### ネットワークの設定
```shell
cd /etc/netplan

sudo mv 00-original.yaml 02-original.yaml

sudo netplan try

sudo systemctl start hostapd

sudo systemctl restart hostapd
```

### micro_ros_agentの起動
```shell
cd ./earth/ros2_ws

colcon build

source ./install/local_setup.zsh

ros2 run micro_ros_agent micro_ros_agent udp4 --port 2000
```

### viewerの起動
```shell
cd ./earth/ros2_ws

source ./install/local_setup.zsh

ros2 launch viewer viewer.launch.py
```

### ログの再生
```shell
ros2 bag play rosbag2_2024_11_09-15_22_09/
```

## ゆめくじらプログラミングコンテスト2024 一般投票賞
![](https://www.wakayama-nct.ac.jp/wp-content/uploads/2024/12/5bd35e9a538ef924bdd9eee30eb179be-scaled-e1733903967315.jpg)

[メディア掲載](https://www.wakayama-nct.ac.jp/cat_topics/8393/)