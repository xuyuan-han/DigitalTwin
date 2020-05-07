# 同济大学中德工程学院机械电子工程专业2017级项目管理课程 —— 基于数字孪生系统的自动化编程

## 一、介绍
  
  > 本项目以数字孪生系统为训练模型，以真实工业机器人及工位作为校验工具。通过虚拟系统中的碰撞预防算法，指导工业机器人完成工件夹取编程。本项目中使用ROS1与MoveIt！包做机器人的路径规划与碰撞预防，采用机器学习算法控制工业机器人六轴转动。项目参与学生需要具备一定的Linux知识以及C++、Python编程基础。

## 二、环境依赖

- Ubuntu 18.04
- ROS1 Melodic
- Moveit！

## 三、软件架构

```Shell
.
├── Documents                 //一些文档资料
├── LICENSE                   //MIT开源协议
├── manage.md                 //Git工作流程说明文档
├── Model                     //存放机器人模型
│   ├── Model_URDF_SLDASM     //kuka机器人SolidWorks模型
│   └── 归档                   //模型归档
├── README.en.md
├── README.md
└── ws_kuka                   //moveit工作区
    └── src                   //ros源代码
        ├── kuka_moveit       //使用MoveIt驱动kuka的包
        └── kuka_urdf         //定义kuka机器人urdf的包
```

## 四、使用说明

```Shell
roscore
cd ws_kuka
catkin_make
cd build
make
roslaunch kuka_moveit demo.launch                   //运行kuka的rviz仿真
rosrun kuka_moveit move_group_interface_tutorial    //运行CPP INTERFACE，用.cpp文件给kuka发送目标位置坐标，在rviz的RvizVisualToolsGui面板中点击next执行下一步动作
```

- 如果`plan`后`一直循环显示plan路径`的话，在`rviz`中取消勾选`Displays->MotionPlanning->Planned Path->Loop Animation`

## 五、参与贡献

| 贡献者 |          邮箱          |
| :----: | :--------------------: |
| 邓中柱 |  dzhongzhu@icloud.com  |
| 韩煦源 | xuyuan.HAN@outlook.com |
| 蒋晗茜 |                        |
|  杨丰  | Jessica.YANG.work@outlook.com|
| 臧浩楠 |                        |
|  周行  |                        |

## 六、待办

1. `move_group_interface_tutorial`中有若干步的目标坐标位置与环境挡板重叠，无法规划路径，需要调节坐标值
2. 尚未完成ros与plc间通过opcua的通讯
3. 尚未完成路径规划的效率评估