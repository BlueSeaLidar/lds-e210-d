#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"

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
	std::string comname = "/dev/ttyCH343USB0";
	int baudrate = 3125000;
	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(comname, baudrate);
	PaceCatLidarSDK::getInstance()->SetLogDataCallBackPtr(devID, LogDataCallback);
	std::string upgrade_file_path = "LDS-E210-D_V1.2_20260113_0.bin";

	bool ret = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf("lidar upgrade %d\n", ret);
	if (ret)
	{
		printf("Upgrade successful, waiting for device reset...\n");
		std::this_thread::sleep_for(std::chrono::seconds(10));

		PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
		std::string ver;
		for (int i = 0; i < 5; i++)
		{
			bool getver_isok = PaceCatLidarSDK::getInstance()->QueryVersion(devID, ver);
			std::cout << getver_isok << " " << ver << std::endl;
			if (getver_isok && !ver.empty())
			{
				printf("Successfully queried version!\n");
				break;
			}
		}
	}
	else
	{
		printf("Upgrade failed!\n");
	}
	PaceCatLidarSDK::getInstance()->Uninit();
	return 0;
}