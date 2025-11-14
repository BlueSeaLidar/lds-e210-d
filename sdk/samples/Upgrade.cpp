#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"


void LogDataCallback(uint16_t handle, const uint8_t dev_type, const char *data, uint16_t len)
{
	if (data == nullptr)
	{
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	std::string comname = "/dev/ttyCH343USB1";
	int baudrate = 3125000;
	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(comname,baudrate);
	PaceCatLidarSDK::getInstance()->SetLogDataCallBackPtr(devID, LogDataCallback);
    std::string  upgrade_file_path = "LDS-E210-D_V0.7_20250916_000000.bin";

	bool ret = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf("lidar upgrade %d\n", ret);
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}