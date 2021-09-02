# Beluga-Client

这是一个和Auto-Beluga感知系统的可视化客户端。

### 编译方式

```bash
$catkin_make -DCATKIN_WHITELIST_PACKAGES="me120_msg" #先编译消息
$catkin_make -DCATKIN_WHITELIST_PACKAGES="" #再编译整个文件夹
```

### 运行方式

```bash
$roslaunch me120_tool demo.launch
```

