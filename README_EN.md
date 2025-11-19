# LDS-E210-D ROS/SDK Driver

----------
The Blue Sea Optoelectronics LDS-E210-D driver is specifically designed for connecting the company's custom LDS-E210-D products and is not compatible with other products. 
The ROS driver can run on operating systems with the ROS environment installed and mainly supports Ubuntu series operating systems (16.04 LTS - 20.04 LTS).
Tested hardware platforms that can run this driver include mainstream Intel x86 CPU platforms and some ARM64 hardware platforms (such as NVIDIA, Rockchip, Raspberry Pi, etc., which may require updating the CH343 driver).

---
## SDK

windows/linux SDK and Demo programs for LDS-E210-D lidar

### HOW TO BUILD AND USE
***EXE file explain:***
>PointCloudAndImu:    Output point cloud data of each frame of lidar
>Upgrade:             lidar Firmware Upgrade 

### LINUX
***Prerequisite: g++ and gcc must be installed***

	cmake CMakeList.txt
	make
	./${EXE file} 		EXE file{PointCloudAndImu,Upgrade}

### WINDOWS

use cmake_gui  build ,open project and compile with visual stdioxx,then enerate a solution

---
## ROS

***Please ensure that the path does not contain Chinese characters, otherwise the compilation will fail!***

### Get Build Run ROS

	git clone https://github.com/BlueSeaLidar/lds-e210-d.git   
	cd lds-e210-d/ros
	catkin_make
	source ./devel/setup.sh
	sudo chmod 777 /dev/ttyCH343USB0 
	roslaunch pacecat_e210_d LDS-E210-D.launch

### Driver Package Launch Configuration File Description:
#ROS#（Required parameters of the framework）
>param name="scan_topic" value="scan" 
param name="cloud_topic" value="cloud" 
param name="frame_id" value="map" 
#DATA#（Drive restriction parameters at the custom data layer）
param name="min_dist" value="0.01"
param name="max_dist" value="50.0"
param name="output_scan" value="true" 
param name="output_cloud" value="false"
param name="output_cloud2" value="false"
#CONNECT#（Parameters for driving and connecting the radar）
param name="type" value="uart" 
param name="com_name" value="/dev/ttyCH343USB0" 
param name="baud_rate" value="3125000" 

---
## Directory file structure

### Table of Contents Explanation

	lds-e210-d/ROS  ROS driver
	lds-e210-d/SDK  SDK driver
	
note:only the demo is different between the two, the underlying structure is identical.

### SDK struct

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

###  ROS struct
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



## Interface description

This section focuses on the interfaces called externally in the PaceCatLidarSDK class.
update time:2025-11-19
***#include<pacecatlidarsdk.h>***

### Initialization and Destruction
```C++

/**
 * @brief Initialize the SDK and allocate the necessary resources
 */
void Init();

/**
 * @brief Deinitialize the SDK, release all resources, and stop all threads
 */
void Uninit();
```

### Lidar Management
#### Add and connect lidar
```C++
/**
 * @brief Add a lidar to the management list
 * @param com_name 串口Serial Port Device Name (e.g., "/dev/ttyUSB0", "COM3")
 * @param baudrate Serial Port Baud Rate (e.g., 115200, 9600)
 * @return int successfully returned the assigned unique lidar ID (>=0)，failure returned -1
 */
int AddLidar(std::string com_name, uint32_t baudrate);

/**
 * @brief Connect the lidar and start the data acquisition thread
 * @param ID lidar ID ( AddLidar return)
 * @return bool successfully connected and returned true，Failure returned false
 */
bool ConnectLidar(uint16_t ID);

/**
 * @brief Disconnect the radar and stop data collection
 * @param ID lidar ID
 * @return bool successfully disconnected and returned true
 */
bool DisconnectLidar(uint16_t ID);
```

#### Information Inquiry
```C++
/**
 * @brief Check lidar serial number
 * @param ID lidar ID
 * @param[out] sn String reference for storing serial numbers
 * @return bool successfully and returned true
 */
bool QuerySN(uint16_t ID, std::string &sn);

/**
 * @brief Check lidar firmware version information
 * @param ID lidar ID
 * @param[out] info A string reference that stores version information
 * @return bool successfully and returned true
 */
bool QueryVersion(uint16_t ID, std::string &info);
```

### Lidar Control
```C++
/**
 * @brief Set lidar operation
 * @param ID lidar ID
 * @param action  operation cmd
 *        0: No operation
 *        1: Start Scan
 *        2: Stop scanning
 *        3: Restart the device
 * @return bool successfully and returned true
 */
bool SetLidarAction(uint16_t ID, int action);

/**
 * @brief Set lidar rotation speed
 * @param ID lidar ID
 * @param rpm rpm value (support 600, 900, 1200)
 * @return bool successfully and returned true
 */
bool SetRPM(uint16_t ID, uint16_t rpm);

/**
 * @brief Clear the frame buffer (applicable when powering on/off or in cases of unstable speed)
 * @param ID lidar ID
 * @return bool successfully and returned true
 */
bool ClearFrameCache(uint16_t ID);
```

### Firmware Upgrade
```C++
/**
 * @brief Perform firmware upgrade
 * @param ID lidar ID
 * @param path Firmware file path
 * @return bool upgrade successful and returned true
 */
bool SetLidarUpgrade(uint16_t ID, std::string path);
```

## Business Support:
For specific usage issues, please contact technical support through the official website.(https://pacecat.com/)