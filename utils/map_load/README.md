点云的地图的发送逻辑中，我发现每次使用rostopic echo 时只会打印一次，然后就不会再打印了。并且rviz中也是始终都会显示的，我想知道这里面具体发生了什么
# latched话题：
你创建的发布者使用了 latching（持久化）机制 (advertise<sensor_msgs::PointCloud2>(map_topic, 10, true) 中的 true 参数）。Latching 意味着最后一次发布的信息会被保存下来，新的订阅者连接时会立即接收到这条消息。因此，每当一个新的订阅者（如 rostopic echo）连接到这个话题时，它都会立即接收到那条被 latched 的消息。


