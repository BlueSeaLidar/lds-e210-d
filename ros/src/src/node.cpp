#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <ros/console.h>
#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <queue>
#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
struct ArgData
{
  std::string frame_id;
  int dev_id;
  bool output_scan;
  bool output_cloud;
  bool output_cloud2;
  // bool with_angle_filter;
  // double min_angle;
  // double max_angle;
  double min_dist;
  double max_dist;
  // std::vector<Range> masks;
  std::string type;
  // int uuid;
  // int rpm;
  std::string cloud_topic;
  std::string scan_topic;
};

ArgData g_argdata;
ros::Publisher laser_pubs;
ros::Time last_stamp;
void PointCloudCallback(uint16_t handle, const uint8_t dev_type, const void *data, uint16_t data_len)
{
  if (data == nullptr)
  {
    return;
  }

  ZhuiMiProtocol::Packet_ZM *head = (ZhuiMiProtocol::Packet_ZM *)data;
  ZhuiMiProtocol::Point_ZM *points = (ZhuiMiProtocol::Point_ZM *)head->pointdata;

  // 获取当前帧的时间戳（ROS 时间）
  ros::Time current_stamp(head->ts_beg[0], head->ts_beg[1]);

  // 打印当前帧和上一帧的时间戳（如果上一帧有效）
  if (last_stamp.isValid())
  {
    // 计算时间差（单位：毫秒）
    double time_diff_ms = (current_stamp - last_stamp).toSec() * 1000.0;
    // 检查间隔是否超过阈值（如80ms）
    if (time_diff_ms > 75.0)
    {
      ROS_WARN("High interval detected: %.2f ms (> 75ms)", time_diff_ms);
      // 打印时间戳和间隔
      ROS_INFO(
          "Timestamp: Prev=%.3f s (Sec: %u, Nsec: %u), Curr=%.3f s (Sec: %u, Nsec: %u), Interval=%.2f ms\n",
          last_stamp.toSec(), last_stamp.sec, last_stamp.nsec,
          current_stamp.toSec(), current_stamp.sec, current_stamp.nsec,
          time_diff_ms);
    }
  }

  // 更新上一帧的时间戳
  last_stamp = current_stamp;

  sensor_msgs::LaserScan msg;
  int N = head->pointnum;
  // make  min_ang max_ang  convert to mask
  msg.header.stamp.sec = head->ts_beg[0];
  msg.header.stamp.nsec = head->ts_beg[1];

  double ti = double(head->ts_beg[0]) + double(head->ts_beg[1]) / 1000000000.0;
  double tx = double(head->ts_end[0]) + double(head->ts_end[1]) / 1000000000.0;

  msg.scan_time = (tx - ti) / 19 * 20;
  msg.time_increment = msg.scan_time / (N - 1);

  // printf("%u %u %u %u\n", head->ts_beg[0],head->ts_beg[1],head->ts_end[0],head->ts_end[1]);
  msg.header.frame_id = g_argdata.frame_id;
  msg.range_min = g_argdata.min_dist;
  msg.range_max = g_argdata.max_dist; // 8.0;

  msg.angle_min = head->start_angle/180.0*M_PI;
  msg.angle_max = msg.angle_min+2*M_PI - (2 * M_PI) / N;
  msg.angle_increment = M_PI * 2 / N;

  msg.intensities.resize(N);
  msg.ranges.resize(N);
  int idx = 0;
  for (int i = 0; i < N; i++)
  {
    double d = points[i].distance / 1000.0;

    if (d == 0 || d > g_argdata.max_dist || d < g_argdata.min_dist)
      msg.ranges[idx] = std::numeric_limits<float>::infinity();
    else
      msg.ranges[idx] = d;

    msg.intensities[idx] = points[i].reflectivity;
    idx++;
  }
  laser_pubs.publish(msg);
}

void LogDataCallback(uint16_t handle, const uint8_t dev_type, const char *data, uint16_t len)
{
  if (data == nullptr)
  {
    return;
  }
  
  if (dev_type == ZhuiMiProtocol::MSG_ALARM)
  {
    ZhuiMiProtocol::UartState uartstate;
    memcpy(&uartstate, data, len);
    if (uartstate.coil_disconnection)
      ROS_ERROR("coil_disconnection");

    if (uartstate.overcurrent)
      ROS_ERROR("overcurrent");

    if (uartstate.encoder_error)
      ROS_ERROR("encoder_error");

    if (uartstate.motor_fault)
      ROS_ERROR("motor_fault");

    if (uartstate.low_time_synchronization_accuracy)
      ROS_ERROR("low_time_synchronization_accuracy");

    if (uartstate.no_gps_signal)
      ROS_ERROR("no_gps_signal");

    if (uartstate.no_pps_signal)
      ROS_ERROR("no_pps_signal");

    if (uartstate.gps_signal_abnormal)
      ROS_ERROR("gps_signal_abnormal");

    if (uartstate.backplane_low_voltage)
      ROS_ERROR("backplane_low_voltage");

    if (uartstate.backplane_high_voltage)
      ROS_ERROR("backplane_high_voltage");

    if (uartstate.head_temperature_abnormal)
      ROS_ERROR("head_temperature_abnormal");

    if (uartstate.head_low_loltage)
      ROS_ERROR("head_low_loltage");

    if (uartstate.head_high_loltage)
      ROS_ERROR("head_high_loltage");

    if (uartstate.head_no_data)
      ROS_ERROR("head_no_data");

    if (uartstate.head_data_abnormal)
      ROS_ERROR("head_data_abnormal");

    if (uartstate.infrared_receive_error)
      ROS_ERROR("infrared_receive_error");
  }
  else
    ROS_ERROR("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lidar_e210_e");
  ros::NodeHandle priv_nh("~");
  ros::NodeHandle node_handle;
  priv_nh.param("frame_id", g_argdata.frame_id, std::string("map"));
  // bool with_angle_filter;
  // double min_angle, max_angle;
  // priv_nh.param("with_angle_filter", argdata.with_angle_filter, false); // true: enable angle filter, false: disable
  // priv_nh.param("min_angle", argdata.min_angle, -M_PI);                 // angle filter's low threshold, default value: -pi
  // priv_nh.param("max_angle", argdata.max_angle, M_PI);                  // angle filters' up threashold, default value: pi

  // range limitation
  double min_dist, max_dist;
  priv_nh.param("min_dist", g_argdata.min_dist, 0.1);  // min detection range, default value: 0M
  priv_nh.param("max_dist", g_argdata.max_dist, 50.0); // max detection range, default value: 9999M

  priv_nh.param("scan_topic", g_argdata.scan_topic, std::string("scan"));
  priv_nh.param("cloud_topic", g_argdata.cloud_topic, std::string("cloud"));
  std::string com_name;
  int baud_rate;
  priv_nh.param("com_name", com_name, std::string("/dev/ttyCH343USB0"));
  priv_nh.param("baud_rate", baud_rate, 3125000);
  priv_nh.param("output_scan", g_argdata.output_scan, true);      // true: enable output angle+distance mode, 0: disable
  priv_nh.param("output_cloud", g_argdata.output_cloud, false);   // false: enable output xyz format, 0 : disable
  priv_nh.param("output_cloud2", g_argdata.output_cloud2, false); // false: enable output xyz format, 0 : disable

  // if (argdata.output_cloud)
  // {
  //   cloud_pubs = node_handle.advertise<sensor_msgs::PointCloud>(argdata.cloud_topic, 50);
  // }
  // if (argdata.output_cloud2)
  // {
  //   cloud2_pubs = node_handle.advertise<sensor_msgs::PointCloud2>(argdata.cloud_topic, 50);
  // }
  if (g_argdata.output_scan)
  {
    laser_pubs = node_handle.advertise<sensor_msgs::LaserScan>(g_argdata.scan_topic, 50);
  }

  PaceCatLidarSDK::getInstance()->Init();
  int devID = PaceCatLidarSDK::getInstance()->AddLidar(com_name, baud_rate);

  PaceCatLidarSDK::getInstance()->SetPointCloudCallBackPtr(devID, PointCloudCallback);
  PaceCatLidarSDK::getInstance()->SetLogDataCallBackPtr(devID, LogDataCallback);
  PaceCatLidarSDK::getInstance()->ConnectLidar(devID);

  while (ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
