pip3 install python-can cantools
pip3 install scikit-learn numpy scipy

录制点云数据：
```clike
sudo tcpdump -i ens33 -w /home/cyun/my_lidar.pcap udp port 6699 or udp port 7788
```
录制can报文：

```clike
candump -l can2
```

```clike
# 1. 加载虚拟CAN模块
sudo modprobe vcan

# 2. 创建一个叫 can2 的虚拟接口 (名字必须和你 python 代码里的一致！)
sudo ip link add dev can2 type vcan

# 3. 启动接口
sudo ip link set up can2
```

```clike
canplayer -I candump-2023-xx-xx_xxxxxx.log
```

```clike
# 格式: canplayer -I <文件> 录制时can=现在想要的输出的can口
canplayer -I my_drive_data.log i can2=can0
```


压力测试

```clike
# -g 10: 间隔10ms发一次
# -I 32A: 指定发送 ID 为 32A (即810)
# -D r: 数据内容随机
cangen can2 -g 10 -I 32A -D r
```

rslidar（以及绝大多数激光雷达）使用的是 UDP 协议 进行大量数据的传输。

```clike
net.core.rmem_max = 26214400
net.core.rmem_default = 26214400
```

