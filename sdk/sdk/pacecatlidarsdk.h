// M300_SDK.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。
#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#include"protocol.h"
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#define E210_D_SDKVERSION "V1.1_2025102001" // SDK版本号

typedef struct
{
	std::string mcu_ver;
	std::string motor_ver;
	std::string software_ver;
}VersionInfo;

//运行配置 
struct RunConfig
{
	uint16_t ID;
	uint16_t handle;
	std::thread  thread_subData;
	LidarCloudPointCallback  cb_cloudpoint;
	LidarLogDataCallback cb_logdata;;
	std::string comname;
	uint32_t baudrate;
	uint32_t frame_cnt;
	int send_len;
	std::string send_buf;
	int recv_len;
	std::string recv_buf;
	int run_state;
	int action;
};

struct CmdTask
{
    uint64_t send_timestamp;//发送的时间戳
    uint8_t tried;//已经尝试次数
    std::string cmd;//测试指令
};
struct CmdTaskList
{
	uint8_t max_waittime;//最大等待时间 单位:秒
	uint8_t max_try_count;//最大重试次数
	std::queue<CmdTask>cmdtask;//任务列表
};


class PaceCatLidarSDK
{
public:
	static PaceCatLidarSDK *getInstance();
	static void deleteInstance();

	void Init();
	void Uninit();
	/*
	 *	callback function  get pointcloud data  logdata
	 */
	bool SetPointCloudCallBackPtr(uint16_t ID, LidarCloudPointCallback cb);
	bool SetLogDataCallBackPtr(uint16_t ID, LidarLogDataCallback cb);

	void WritetPointCloudCallBack(uint16_t ID, const uint8_t msg_type, const void *data, uint16_t data_len);
	void WriteLogDataCallBack(uint16_t ID, const uint8_t msg_type, const char *data, uint16_t data_len);

	/*
	 *	add lidar by com name and bandrate
	 */
	int AddLidar(std::string com_name,uint32_t bandrate);
	/*
	 *	connect lidar     send cmd/parse recvice data
	 */
	bool ConnectLidar(uint16_t ID);
	/*
	 *	disconnect lidar,cancel listen lidar
	 */
	bool DisconnectLidar(uint16_t ID);
	/*
	 *	query connect lidar base info
	 */
	bool QuerySN(uint16_t ID, std::string &sn);
	/*
	 *	query connect lidar version
	 */
	bool QueryVersion(uint16_t ID, std::string &info);

	/*
	 *	set lidar    start work     stop work（sleep）  restart work(reboot)
	 */
	bool SetLidarAction(uint16_t ID, int action);
	/*
	 *	set lidar    rpm(support 600 900 1200)
	 */
	bool SetRPM(uint16_t ID, uint16_t rpm);
	/*
	 *	set lidar  firmware upgrade
	 */
	bool SetLidarUpgrade(uint16_t ID, std::string path);
	/*
	 *	clear frame cache (Applied to situations   powered on or off, or rpm is unstable)
	 */
	bool ClearFrameCache(uint16_t ID);
	
private:
	void UartThreadProc(uint16_t id);
	RunConfig* GetConfig(uint16_t ID);
	int parsePointCloud(int ID,uint8_t *data, uint16_t len,int32_t &sequence_num,uint64_t &last_timestamp,uint16_t &last_start_angle,int &first_start_angle,uint16_t &consume,std::vector<ZhuiMiProtocol::Point_ZM>&points,ZhuiMiProtocol::Packet_ZM&packet_frame);

	uint8_t* load_bin(const char* path, size_t& len);
	int XReadFile(int fd, char* line, int sz);
	int xymodem_send(int serial_fd, size_t len, const uint8_t* buf, int protocol, int wait);
	int xmodem_send(int serial_fd, size_t len, const uint8_t* buf );
	int xread(int fd, void *buf, size_t sz);
	int xwrite(int fd, void *buf, size_t sz);
	
private:
	static PaceCatLidarSDK *m_sdk;
	PaceCatLidarSDK();
	~PaceCatLidarSDK();
	int m_lidar_id_counter{0};
	std::vector<RunConfig*> m_lidars;
};

bool uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch,int waittime);






