# CyberDog 实时定位模块

---
## **[简介]**

本模块将开源算法open_vins的定位相关部分移植到铁蛋开源平台上以供开发者使用，并根据使用需求加入了唤醒/挂起、定位效果变差时切换到里程计输出等辅助功能。
具体接口和功能特性请参照开源框架open_vins(https://docs.openvins.com/)


## **运行 & 调试**

### **启动系统运行**

启动系统运行即使用`ros2 launch`启动，铁蛋会开机自动调用该脚本启动，除了SLAM节点外，还同时启动其他若干个节点，具体可参考[athena_bringup](../athena_bringup)进行深入了解。

单目SLAM启动指令为：

```
ros2 launch ov_msckf ros2.mono.launch.py
```

双目SLAM启动指令为（此时realsense进入emittor_on_off模式，左右目图像以15fps输出正常灰度图像）：

```
ros2 launch ov_msckf ros2.launch.py
```

该启动状态是机器人正常启动的流程，可以测试所有功能。

### **调试方法**

本模块支持GDB调试。

#### GDB调试

1. 首先需要在`CMakeLists.txt`中添加`-g`的编译标记，一般在`add_compile_options`函数里
2. 再根据[athena_bringup](../athena_bringup)进行修改，使用`gdb`前缀进行启动
3. 确定系统中是否具备调试终端工具，包括`gdb`和`xterm`等，如不具备需要安装
4. 确保系统中在相同`Domain ID`和相同`namespace`下不存在相同的节点后，在具备图形化界面的环境使用`Launch`进行启动。
5. 开始调试。

## **[未来优化]**

- 支持多目vio
- 优化初始化功能 
- 紧耦合里程计提高定位精度 正常