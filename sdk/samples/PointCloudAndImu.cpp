/*If you are using a third-party queue library instead of the queue wheel in the demo, just replace the contents of the macro definition CUSTOM_WHELL*/

#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
#include <sstream>
#define CUSTOM_WHELL

#ifdef CUSTOM_WHELL
#include "../3rdparty/readerwriterqueue/readerwriterqueue.h"
using namespace moodycamel;
ReaderWriterQueue<std::string> *g_pointcloud_queue;

#endif // DEBUG

void PointCloudCallback(uint16_t handle, const uint8_t dev_type, const void *data, uint16_t data_len)
{
	if (data == nullptr)
	{
		return;
	}
#ifdef CUSTOM_WHELL
	std::string chunk((char *)data, data_len);
	g_pointcloud_queue->try_enqueue(chunk);
#endif
}

void LogDataCallback(uint16_t handle, const uint8_t dev_type, const char *data, uint16_t len)
{
  if (data == nullptr)
  {
    return;
  }
  if(dev_type==ZhuiMiProtocol::MSG_ALARM)
  {
    ZhuiMiProtocol::UartState uartstate;
    memcpy(&uartstate,data,len);
    if(uartstate.coil_disconnection)
      printf("coil_disconnection\n");

    if(uartstate.overcurrent)
      printf("overcurrent\n");

    if(uartstate.encoder_error)
      printf("encoder_error\n");

    if(uartstate.motor_fault)
      printf("motor_fault\n");

    if(uartstate.low_time_synchronization_accuracy)
      printf("low_time_synchronization_accuracy\n");

    if(uartstate.no_gps_signal)
      printf("no_gps_signal\n");

    if(uartstate.no_pps_signal)
      printf("no_pps_signal\n");

    if(uartstate.gps_signal_abnormal)
      printf("gps_signal_abnormal\n");

    if(uartstate.backplane_low_voltage)
      printf("backplane_low_voltage\n");

    if(uartstate.backplane_high_voltage)
      printf("backplane_high_voltage\n");

    if(uartstate.head_temperature_abnormal)
      printf("head_temperature_abnormal\n");

    if(uartstate.head_low_loltage)
      printf("head_low_loltage\n");

    if(uartstate.head_high_loltage)
      printf("head_high_loltage\n");

    if(uartstate.head_no_data)
      printf("head_no_data\n");

    if(uartstate.head_data_abnormal)
      printf("head_data_abnormal\n");

    if(uartstate.infrared_receive_error)
      printf("infrared_receive_error\n");
  }
  else
    printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}


int main()
{
#ifdef CUSTOM_WHELL
	ReaderWriterQueue<std::string> q(20);
	g_pointcloud_queue = &q;
#endif
	std::string com_name = "/dev/ttyCH343USB1";
	int baud_rate = 3125000;
	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(com_name,baud_rate);

	PaceCatLidarSDK::getInstance()->SetPointCloudCallBackPtr(devID, PointCloudCallback);
	PaceCatLidarSDK::getInstance()->SetLogDataCallBackPtr(devID, LogDataCallback);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);

	// bool stop_isok=PaceCatLidarSDK::getInstance()->SetLidarAction(devID,ZhuiMiProtocol::STOP);
	// bool start_isok=PaceCatLidarSDK::getInstance()->SetLidarAction(devID,ZhuiMiProtocol::START);
	// bool setrpm_isok=PaceCatLidarSDK::getInstance()->SetRPM(devID,1200);
	std::string sn;
	bool getsn_isok=PaceCatLidarSDK::getInstance()->QuerySN(devID,sn);
	std::string ver;
	bool getver_isok=PaceCatLidarSDK::getInstance()->QueryVersion(devID,ver);
	std::cout<<getsn_isok<<" "<<sn<<" "<<getver_isok<<" "<<ver<<std::endl;
	//std::cout<<stop_isok<<start_isok<<setrpm_isok<<getsn_isok<<sn<<std::endl;
	while (1)
	{
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifdef CUSTOM_WHELL
		std::string chunk;
		bool ret = g_pointcloud_queue->try_dequeue(chunk);
		if (ret)
		{
			ZhuiMiProtocol::Packet_ZM *data = (ZhuiMiProtocol::Packet_ZM *)(chunk.c_str());
			std::ostringstream oss;
			oss << std::this_thread::get_id();
			// printf("data:main thread:%s timestamp:%d/%d/%d %d:%d:%d %d\n",
			// oss.str().c_str(), 
			// data->date_time[0], data->date_time[1], data->date_time[2],
			// data->date_time[3],data->date_time[4],data->date_time[5],
			// data->timestamp);
			//第一个包的起始时间戳和最后一个包的起始时间戳，如果要算帧时间，需要time/19*20
			printf("data:main thread:%s timestamp:%u %u %u %u \n",
			oss.str().c_str(), 
			data->ts_beg[0],data->ts_beg[1],data->ts_end[0],data->ts_end[1]);
			//距离值大于65533的值为特殊值  65533-65535 近距离盲点 距离异常 远距离忙点
			// for (uint16_t i = 0; i < data->pointnum; i++) {
			// 	printf("%d %d %d\n", i,data->pointdata[i].distance,data->pointdata[i].reflectivity);
			// }
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}
