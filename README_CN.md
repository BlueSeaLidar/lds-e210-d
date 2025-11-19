# 蓝海光电LDS-E210-D ROS/SDK驱动程序

## 概述

----------
蓝海光电lds-e210-d驱动程序是专门用于连接本公司特殊定制的的LDS-E210-D产品使用，与其它产品不兼容。
其中Ros驱动程序可以在安装了ROS环境的操作系统中运行，主要支持ubuntu系列操作系统（16.04LTS-20.04LTS）。
经测试可以运行该驱动程序的硬件平台包括：intel x86 主流 cpu 平台，部分 ARM64 硬件平台（如 英伟达、瑞芯微，树莓派等，可能需要更新ch343驱动）。

---
## SDK

SDK和demon用于LDS-E210-D激光雷达在windows和linux上演示运行

***samples文件说明：***

>PointCloudAndImu：输出每帧激光雷达的点云数据
>Upgrade：激光雷达固件升级

### LINUX

***先决条件：必须安装g++和gcc***

    cmake CMakeList.txt
    make
    ./${EXE file}		EXE file{PointCloudAndImu,Upgrade}



### WINDOWS

使用cmake_gui构建，打开项目并使用Visual Studio xx编译，然后生成一个可执行文件

---

## ROS

***请确保路径中不包含中文字符，否则编译会失败！***

### 获取构建运行ROS驱动包

    git clone https://github.com/BlueSeaLidar/lds-e210-d.git   
    cd lds-e210-d/ros
    catkin_make
    source ./devel/setup.sh
    sudo chmod 777 /dev/ttyCH343USB0 
    roslaunch pacecat_e210_d LDS-E210-D.launch


### 驱动包launch配置文件说明

#ROS#（框架必须的参数）
>param name="scan_topic" value="scan" #发布激光雷达话题  
param name="cloud_topic" value="cloud" #发布点云话题  
param name="frame_id" value="map" #标志坐标系的名称
#DATA#（驱动自定义数据层面的限制参数）
param name="min_dist" value="0.01"#最小点云距离(m)
param name="max_dist" value="50.0"#最大点云距离(m)
param name="output_scan" value="true" #二维扫描数据(默认)
param name="output_cloud" value="false"#三维空间数据
param name="output_cloud2" value="false"#三维空间数据
#CONNECT#（驱动连接雷达的参数）
param name="type" value="uart" 
param name="com_name" value="/dev/ttyCH343USB0" 
param name="baud_rate" value="3125000" 

---

## 目录文件结构

### 目录说明

	lds-e210-d/ROS  ROS驱动
	lds-e210-d/SDK  SDK驱动
	
说明:二者仅demo层有区别，底层结构完全相同

### SDK结构
```C++
├── samples                     
│   ├──PointCloudAndImu.cpp
│   └──Upgrade.cpp
├── sdk
│   ├── define.h
│   ├── protocol.h
│   ├── global.h
│   ├── global.cpp
│   ├── pacecatlidarsdk.h
│   ├── pacecatlidarsdk.cpp
│   └── uart.c
├── CMakeLists.txt
```
	例子demo
		点云数据demo
		固件升级demo
	核心sdk
		宏定义和公共基础头文件
		雷达协议
		通用模块函数
		sdk类以及对外开放接口
	cmake工程文件

### ROS结构
```C++
├── launch
│   └──LDS-E210-D.launch
├── msg
│   ├──CustomPoint.msg
│   └──CustomMsg.msg
├── rviz
│   ├──demo.rviz
│   └──1.rviz
├── src
│   └──node.cpp
├── sdk
│   ├── define.h
│   ├── protocol.h
│   ├── global.h
│   ├── global.cpp
│   ├── pacecatlidarsdk.h
│   ├── pacecatlidarsdk.cpp
│   └── uart.c
├── CMakeLists.txt
├── package.xml
```
	运行脚本文件
		指定雷达型号脚本
	消息格式文件
		自定义点云格式
		自定义包数据格式
	可视化脚本文件
		默认运行脚本
	节点运行文件
		主程序入口
	核心sdk
		宏定义和公共基础头文件
		雷达协议
		通用模块函数
		sdk类以及对外开放接口
	cmake工程文件
	包模块管理文件


---
## 接口说明

这里主要介绍 PaceCatLidarSDK类中对外调用的接口。
更新时间:2025-11-19
***#include<pacecatlidarsdk.h>***

### 单例初始化与销毁
```C++

/**
 * @brief 初始化 SDK，分配必要资源
 */
void Init();

/**
 * @brief 反初始化 SDK，释放所有资源并停止所有线程
 */
void Uninit();
```

### 雷达管理
#### 添加与连接雷达
```C++
/**
 * @brief 添加一个雷达设备到管理列表
 * @param com_name 串口设备名 (e.g., "/dev/ttyUSB0", "COM3")
 * @param baudrate 串口波特率 (e.g., 115200, 9600)
 * @return int 成功返回分配的唯一雷达ID (>=0)，失败返回-1
 */
int AddLidar(std::string com_name, uint32_t baudrate);

/**
 * @brief 连接雷达并启动数据采集线程
 * @param ID 雷达ID (由 AddLidar 返回)
 * @return bool 成功连接返回 true，失败返回 false
 */
bool ConnectLidar(uint16_t ID);

/**
 * @brief 断开雷达连接并停止数据采集
 * @param ID 雷达ID
 * @return bool 成功断开返回 true
 */
bool DisconnectLidar(uint16_t ID);
```

#### 信息查询
```C++
/**
 * @brief 查询雷达序列号
 * @param ID 雷达ID
 * @param[out] sn 存储序列号的字符串引用
 * @return bool 成功返回 true
 */
bool QuerySN(uint16_t ID, std::string &sn);

/**
 * @brief 查询雷达固件版本信息
 * @param ID 雷达ID
 * @param[out] info 存储版本信息的字符串引用
 * @return bool 成功返回 true
 */
bool QueryVersion(uint16_t ID, std::string &info);
```

### 雷达控制
```C++
/**
 * @brief 设置雷达操作
 * @param ID 雷达ID
 * @param action 动作指令 
 *        0: 无操作
 *        1: 启动扫描
 *        2: 停止扫描
 *        3: 重启设备
 * @return bool 成功返回 true
 */
bool SetLidarAction(uint16_t ID, int action);

/**
 * @brief 设置雷达转速
 * @param ID 雷达ID
 * @param rpm 转速值 (支持 600, 900, 1200)
 * @return bool 成功返回 true
 */
bool SetRPM(uint16_t ID, uint16_t rpm);

/**
 * @brief 清空帧缓存（适用于上电、掉电或转速不稳定情况）
 * @param ID 雷达ID
 * @return bool 成功返回 true
 */
bool ClearFrameCache(uint16_t ID);
```

### 固件升级
```C++
/**
 * @brief 执行固件升级
 * @param ID 雷达ID
 * @param path 固件文件路径
 * @return bool 升级成功返回 true
 */
bool SetLidarUpgrade(uint16_t ID, std::string path);
```

## 商务支持
具体使用问题请通过官网联系技术支持(https://pacecat.com/)