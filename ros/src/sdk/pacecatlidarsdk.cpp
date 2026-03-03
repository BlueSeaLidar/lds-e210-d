// M300_SDK.cpp: 定义应用程序的入口点。
//
#include <fstream>
#include <chrono>
#include "global.h"
#include "pacecatlidarsdk.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>

#define MAX_CMD_BUFFER 1024

PaceCatLidarSDK *PaceCatLidarSDK::m_sdk = new (std::nothrow) PaceCatLidarSDK();
PaceCatLidarSDK *PaceCatLidarSDK::getInstance()
{
	return m_sdk;
}

void PaceCatLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}
PaceCatLidarSDK::PaceCatLidarSDK()
{
}
PaceCatLidarSDK::~PaceCatLidarSDK()
{
}
bool PaceCatLidarSDK::SetPointCloudCallBackPtr(uint16_t ID, LidarCloudPointCallback cb)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_cloudpoint = cb;
	return true;
}

bool PaceCatLidarSDK::SetLogDataCallBackPtr(uint16_t ID, LidarLogDataCallback cb)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_logdata = cb;
	return true;
}

void PaceCatLidarSDK::WritetPointCloudCallBack(uint16_t ID, const uint8_t dev_type, const void *data, uint16_t data_len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, data_len);
}
void PaceCatLidarSDK::WriteLogDataCallBack(uint16_t ID, const uint8_t dev_type, const char *data, uint16_t data_len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_logdata != nullptr)
		lidar->cb_logdata(ID, dev_type, data, data_len);
}

void PaceCatLidarSDK::Init()
{
}
void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = Protocol::QUIT;
	}
}
int PaceCatLidarSDK::AddLidar(std::string com_name, uint32_t bandrate)
{
	RunConfig *cfg = new RunConfig;
	cfg->comname = com_name;
	cfg->baudrate = bandrate;
	cfg->ID = m_lidar_id_counter++;
	cfg->run_state = Protocol::ONLINE;
	cfg->frame_cnt = 0;
	cfg->cb_cloudpoint = NULL;
	cfg->cb_logdata = NULL;

	// printf("%s %d\n",com_name.c_str(), bandrate);
	int fd = SystemAPI::open_serial_port(com_name.c_str(), bandrate);
	if (fd <= 0)
	{
		return false;
	}
	cfg->handle = fd;

	m_lidars.push_back(cfg);
	return cfg->ID;
}
bool PaceCatLidarSDK::ConnectLidar(uint16_t ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if(lidar->handle <= 0)
		lidar->handle = SystemAPI::open_serial_port(lidar->comname.c_str(),lidar->baudrate);
	if (lidar->handle <= 0)
		return false;


	lidar->thread_subData = std::thread(&PaceCatLidarSDK::UartThreadProc, PaceCatLidarSDK::getInstance(), ID);
	lidar->thread_subData.detach();
	return true;
}
bool PaceCatLidarSDK::DisconnectLidar(uint16_t ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != Protocol::QUIT)
		{
			m_lidars.at(i)->run_state = Protocol::OFFLINE;
			return true;
		}
	}
	return false;
}

bool PaceCatLidarSDK::QuerySN(uint16_t ID, std::string &sn)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	lidar->send_buf = "LUUIDH";
	lidar->action = Protocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != Protocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == Protocol::FINISH)
	{
		// printf("%s %d %s\n", __FUNCTION__, __LINE__,lidar->recv_buf.c_str());
		lidar->action = Protocol::NONE;
		sn = lidar->recv_buf;
		// printf("%s %d %s\n", __FUNCTION__, __LINE__,lidar->recv_buf.c_str());

		return true;
	}
	return false;
}
bool PaceCatLidarSDK::QueryVersion(uint16_t ID, std::string &info)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	lidar->send_buf = "LXVERH";
	lidar->action = Protocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != Protocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == Protocol::FINISH)
	{
		lidar->action = Protocol::NONE;
		info = lidar->recv_buf;
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::SetLidarAction(uint16_t ID, int action)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	if (action == Protocol::START)
		lidar->send_buf = "LSTARH";
	else if (action == Protocol::STOP)
		lidar->send_buf = "LSTOPH";
	else
		return false;

	lidar->action = Protocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != Protocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == Protocol::FINISH)
	{
		lidar->action = Protocol::NONE;
		// printf("%s %d %s\n", __FUNCTION__, __LINE__, lidar->recv_buf.c_str());

		if (lidar->recv_buf == "OK")
			return true;
	}
	return false;
}
bool PaceCatLidarSDK::SetRPM(uint16_t ID, uint16_t rpm)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	lidar->send_buf = "LSRPM:" + std::to_string(rpm) + "H";
	lidar->action = Protocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != Protocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == Protocol::FINISH)
	{
		lidar->action = Protocol::NONE;
		if (lidar->recv_buf == "OK")
			return true;
	}
	return false;
}
bool PaceCatLidarSDK::SetLidarUpgrade(uint16_t ID, std::string path)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;
	char log_buf[1024] = {0};
	int log_buf_nr = 0;
	log_buf_nr = sprintf(log_buf, "start upgrade:com_name:%s baudrate:%d path:%s", lidar->comname.c_str(), lidar->baudrate, path.c_str());
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, log_buf, log_buf_nr);

	size_t len = 0;
	uint8_t *file_buf = load_bin(path.c_str(), len);
	if (!file_buf)
	{
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_LOAD_FILE_NG, strlen(ZM_LOAD_FILE_NG));
		return false;
	}
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_LOAD_FILE_OK, strlen(ZM_LOAD_FILE_OK));
	int h = SystemAPI::open_serial_port(lidar->comname.c_str(), lidar->baudrate);
	if (h <= 0)
	{
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_OPEN_COM_NG, strlen(ZM_OPEN_COM_NG));
		return false;
	}
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_OPEN_COM_OK, strlen(ZM_OPEN_COM_OK));
	int max_outtime = 500;		 // 超时时间ms
	int max_trytime = 10;		 // 重试次数
	int one_subpacket_len = 128; // 固件数据传输单次字节128
	int recv_len = 0;
	unsigned char recv_buf[1024] = {0};
	int recv_code;

	// 先切换雷达的波特率
	unsigned char cmd_buf[512] = {0};
	Protocol::TX_CmdHeader *tx_cmd = (Protocol::TX_CmdHeader *)cmd_buf;
	tx_cmd->head[0] = 0x4c;
	tx_cmd->head[1] = 0x48;
	tx_cmd->size = 5 + 4;
	tx_cmd->cmd = Protocol::ZM_CMD_TYPE_SET_BPS;
	uint32_t cmd_bps = 115200;
	memcpy(tx_cmd->data, &cmd_bps, sizeof(uint32_t));

	uint32_t crc = BaseAPI::stm32crc_8(cmd_buf, tx_cmd->size);
	memcpy(cmd_buf + tx_cmd->size, &crc, sizeof(uint32_t));
	// 该命令没有返回值,但是可以用连续的两个报警包判定是否是处于115200的波特率状态下
	bool isok = false;
	for (int i = 10; i >= 0; i--)
	{
		write(h, cmd_buf, tx_cmd->size + 4);
		SystemAPI::closefd(h, false);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		h = SystemAPI::open_serial_port(lidar->comname.c_str(), 115200);
		if (checkBPS_isok(h))
		{
			isok = true;
			break;
		}
		SystemAPI::closefd(h, false);
		h = SystemAPI::open_serial_port(lidar->comname.c_str(), lidar->baudrate);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_CMD_TYPE_SET_BPS_NG_TRY, strlen(ZM_CMD_TYPE_SET_BPS_NG_TRY));
	}
	if (!isok)
	{
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_CMD_TYPE_SET_BPS_NG, strlen(ZM_CMD_TYPE_SET_BPS_NG));
		return false;
	}
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_SET_BPS_OK, strlen(ZM_CMD_TYPE_SET_BPS_OK));

	// 通知雷达开始升级
	tx_cmd->size = 5;
	tx_cmd->cmd = Protocol::ZM_CMD_TYPE_START_UPDATE;
	crc = BaseAPI::stm32crc_8(cmd_buf, tx_cmd->size);
	memcpy(cmd_buf + tx_cmd->size, &crc, sizeof(uint32_t));

	recv_code = readbuf(h, 13, MAX_CMD_BUFFER, max_outtime, max_trytime, Protocol::ZM_CMD_TYPE_START_UPDATE, cmd_buf, tx_cmd->size + 4, recv_buf, recv_len);
	if (recv_code != Protocol::ZM_ACK_TYPE_OK)
	{
		// MYLOG<<"ZM_CMD_TYPE_START_UPDATE:"<<recv_code;
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_CMD_TYPE_START_UPDATE_NG, strlen(ZM_CMD_TYPE_START_UPDATE_NG));
		SystemAPI::closefd(h, false);
		return false;
	}
	// MYLOG<<"ZM_CMD_TYPE_START_UPDATE:OK";
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_START_UPDATE_OK, strlen(ZM_CMD_TYPE_START_UPDATE_OK));

	// 传输固件总大小和CRC
	tx_cmd->size = 5 + 8;
	tx_cmd->cmd = Protocol::ZM_CMD_TYPE_FIRMWARE_INFO;

	uint32_t file_len = len;
	memcpy(&tx_cmd->data[0], &file_len, sizeof(uint32_t));
	uint32_t crc_file = BaseAPI::stm32crc_8(file_buf, file_len);
	memcpy(&tx_cmd->data[4], &crc_file, sizeof(uint32_t));

	crc = BaseAPI::stm32crc_8(cmd_buf, tx_cmd->size);
	memcpy(cmd_buf + tx_cmd->size, &crc, sizeof(uint32_t));

	recv_code = readbuf(h, 13, MAX_CMD_BUFFER, max_outtime, max_trytime, Protocol::ZM_CMD_TYPE_FIRMWARE_INFO, cmd_buf, tx_cmd->size + 4, recv_buf, recv_len);
	if (recv_code != Protocol::ZM_ACK_TYPE_OK)
	{
		// MYLOG<<"ZM_CMD_TYPE_FIRMWARE_INFO:"<<recv_code;
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_FIRMWARE_INFO_NG, strlen(ZM_CMD_TYPE_FIRMWARE_INFO_NG));
		SystemAPI::closefd(h, false);
		return false;
	}
	// MYLOG<<"ZM_CMD_TYPE_FIRMWARE_INFO:OK";
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_FIRMWARE_INFO_OK, strlen(ZM_CMD_TYPE_FIRMWARE_INFO_OK));

	// 传输固件数据
	tx_cmd->size = 5 + 4 + 128;
	tx_cmd->cmd = Protocol::ZM_CMD_TYPE_FIRMWARE_DATA;

	int packet_num = file_len % one_subpacket_len == 0 ? file_len / one_subpacket_len : file_len / one_subpacket_len + 1;
	// MYLOG<<"packet sum:"<<packet_num;
	// 因为file_buf的总大小为1MB，且初始化已经全部置为0,所以最后一个包多出来字节的时候，直接拷贝，不会越界以及会自动补零
	for (int i = 0; i < packet_num; i++)
	{
		memcpy(&tx_cmd->data[0], &i, sizeof(uint32_t));
		memcpy(&tx_cmd->data[4], &file_buf[i * one_subpacket_len], one_subpacket_len);

		crc = BaseAPI::stm32crc_8(cmd_buf, tx_cmd->size);
		memcpy(cmd_buf + tx_cmd->size, &crc, sizeof(uint32_t));
		recv_code = readbuf(h, 13, MAX_CMD_BUFFER, max_outtime, max_trytime, Protocol::ZM_CMD_TYPE_FIRMWARE_DATA, cmd_buf, tx_cmd->size + 4, recv_buf, recv_len);
		float rate = 100.0 * i / packet_num;
		if (recv_code != Protocol::ZM_ACK_TYPE_OK)
		{
			// MYLOG<<"ZM_CMD_TYPE_FIRMWARE_DATA:"<<recv_code<<" "<<i;
			WriteLogDataCallBack(lidar->ID, Protocol::MSG_ERROR, ZM_CMD_TYPE_FIRMWARE_DATA_NG, strlen(ZM_CMD_TYPE_FIRMWARE_DATA_NG));
			SystemAPI::closefd(h, false);
			return false;
		}
		log_buf_nr = sprintf(log_buf, "sync process:%f", rate);
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, log_buf, log_buf_nr);
	}
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_FIRMWARE_DATA_OK, strlen(ZM_CMD_TYPE_FIRMWARE_DATA_OK));

	// 查询固件升级状态
	tx_cmd->size = 5;
	tx_cmd->cmd = Protocol::ZM_CMD_TYPE_END_UPDATE;

	crc = BaseAPI::stm32crc_8(cmd_buf, tx_cmd->size);
	memcpy(cmd_buf + tx_cmd->size, &crc, sizeof(uint32_t));
	recv_code = readbuf(h, 13, MAX_CMD_BUFFER, max_outtime, max_trytime, Protocol::ZM_CMD_TYPE_END_UPDATE, cmd_buf, tx_cmd->size + 4, recv_buf, recv_len);
	if (recv_code != Protocol::ZM_ACK_TYPE_OK)
	{
		// MYLOG<<"ZM_CMD_TYPE_END_UPDATE:"<<recv_code;
		WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_END_UPDATE_NG, strlen(ZM_CMD_TYPE_END_UPDATE_NG));
		SystemAPI::closefd(h, false);
		return false;
	}
	WriteLogDataCallBack(lidar->ID, Protocol::MSG_WARM, ZM_CMD_TYPE_END_UPDATE_OK, strlen(ZM_CMD_TYPE_END_UPDATE_OK));
	SystemAPI::closefd(h, false);
	lidar->handle = 0;
	return true;
}
bool PaceCatLidarSDK::ClearFrameCache(uint16_t ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->action = Protocol::CACHE_CLEAR;
	int index = CMD_REPEAT;
	while (lidar->action != Protocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == Protocol::FINISH)
	{
		lidar->action = Protocol::NONE;
		return true;
	}
	return false;
}

RunConfig *PaceCatLidarSDK::GetConfig(uint16_t ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			return m_lidars[i];
		}
	}
	return nullptr;
}
void PaceCatLidarSDK::UartThreadProc(uint16_t id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;
	unsigned char *recv_buf = (unsigned char *)malloc(BUF_SIZE);
	// 一帧最多3600个点
	uint16_t max_frame_len = sizeof(Protocol::Packet_ZM) + sizeof(Protocol::Point_ZM) * 3600;
	Protocol::Packet_ZM *packet_frame = (Protocol::Packet_ZM *)malloc(max_frame_len);

	char log_buf[1024] = {0};
	int buf_len = 0;
	sprintf(log_buf, "wait lidar data:com_name:%s baudrate:%d ...", cfg->comname.c_str(), cfg->baudrate);
	WriteLogDataCallBack(cfg->ID, Protocol::MSG_WARM, log_buf, strlen(log_buf));
	std::vector<Protocol::Point_ZM> points;
	points.reserve(3600);
	// uint8_t frame_flag = 0;	   // 0/1/2   head body foot
	int32_t sequence_num = -1;	   // 包下标
	uint64_t last_timestamp = 0;   // 包的时间戳
	uint16_t last_start_angle = 0; // 包的角度

	int first_start_angle = -1;		   // 包的起始统计角度
	bool frame_checker = false;		   // 检测点云帧是否长时间丢失
	uint64_t last_frame_timestamp = 0; // 记录上一帧数据的时间戳
	uint64_t last_span_timestamp = 0;  // 记录上一扇区数据的时间戳

	bool is_first_frame = true;
	CmdTaskList cmdtasklist;
	cmdtasklist.max_try_count = 5;
	cmdtasklist.max_waittime = 2;
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LUUIDH"});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LXVERH"});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LSTOPH"});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LSTARH"});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LSRPM:900H"});
	while (cfg->run_state != Protocol::QUIT)
	{
		if (cfg->run_state == Protocol::BUSY)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		// 任务分发
		if (cfg->action == Protocol::CMD_TALK)
		{
			cmdtasklist.cmdtask.push(CmdTask{0, 0, cfg->send_buf.c_str()});
			cfg->action = Protocol::NONE;
		}
		else if (cfg->action == Protocol::CACHE_CLEAR)
		{
			points.clear();
			sequence_num = 0;
			memset(packet_frame, 0, sizeof(Protocol::Packet_ZM));
			cfg->action = Protocol::FINISH;
		}
		// 指令任务处理
		if (!cmdtasklist.cmdtask.empty())
		{
			CmdTask &cmdtask = cmdtasklist.cmdtask.front();
			// 超过最大检测次数
			if (cmdtask.tried > cmdtasklist.max_try_count)
			{
				cmdtasklist.cmdtask.pop();
				continue;
			}
			uint64_t timestamp = SystemAPI::GetTimeStamp(true);
			// 如果发送时间为0或者与当前时间超过最大间隔重发
			if (cmdtask.send_timestamp == 0 || (timestamp - cmdtask.send_timestamp > cmdtasklist.max_waittime * 1000))
			{
				sprintf(log_buf, "%s %lu  time:%lu %lu %d\n", cmdtask.cmd.c_str(), cmdtask.cmd.size(), timestamp, cmdtask.send_timestamp, cmdtask.tried);
				WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));

				write(cfg->handle, cmdtask.cmd.c_str(), cmdtask.cmd.size());
				cmdtask.send_timestamp = timestamp;
				cmdtask.tried++;
			}
		}
		// 每秒判定点云数据是否存在
		uint64_t timestamp = SystemAPI::GetTimeStamp(true);
		// printf("%lu  %lu  %d\n",last_frame_timestamp,timestamp,timestamp-last_frame_timestamp);
		if ((last_frame_timestamp != 0) && timestamp - last_frame_timestamp > 1000)
		{
			sprintf(log_buf, "1s no frame data");
			WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
			last_frame_timestamp = timestamp;
		}
		if (last_span_timestamp != 0 && timestamp - last_span_timestamp > 1000)
		{
			sprintf(log_buf, "1s no span data");
			WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
			last_span_timestamp = timestamp;
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->handle, &fds);
		struct timeval to = {1, 0};
		int ret = select(cfg->handle + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			sprintf(log_buf, "select error,thread end");
			WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
			return;
		}
		else if (ret == 0)
		{
			sprintf(log_buf, "no data");
			WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		if (cfg->handle > 0 && FD_ISSET(cfg->handle, &fds))
		{
			int nr = read(cfg->handle, recv_buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				// sprintf(log_buf, "cache len: %d  read len: %d", buf_len, nr);
				// WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
				// break;
				continue;
			}
			if (nr == 0)
			{
				continue;
			}
			if (nr > 0)
			{
				// printf("%d %d\n",buf_len,nr);
				buf_len += nr;
				if (buf_len >= (BUF_SIZE / 2))
				{
					sprintf(log_buf, "cache is too many:%d %d", buf_len, nr);
					WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
				}
			}
		}
		if (buf_len > 0)
		{
			uint16_t consume = 0;
			bool isfind = false;
			uint16_t last_consume = 0;
			uint16_t idx = 0;
			while (1)
			{
				idx = consume;
				for (idx; idx <= buf_len - 12; idx++)
				{
					// 数据包
					if (recv_buf[idx + 0] == 0xEE && recv_buf[idx + 1] == 0xFF)
					{
						// printf("%s %d\n", __FUNCTION__, __LINE__);
						int ret = parsePointCloud(cfg->ID, (uint8_t *)&recv_buf[idx], buf_len - idx, sequence_num, last_timestamp, last_start_angle, first_start_angle, consume, points, *packet_frame);
						if (ret == 0)
						{
							consume = idx;
							break;
						}
						if (ret == 2)
						{
							if (is_first_frame)
							{
								is_first_frame = false;
								sprintf(log_buf, "PaceCat sdk start work");
								WriteLogDataCallBack(cfg->ID, Protocol::MSG_WARM, log_buf, strlen(log_buf));
							}
							// printf("%s %d %d %d\n", __FUNCTION__, __LINE__,points.size(),consume);
							packet_frame->start_angle = first_start_angle;
							// 帧数据全为0的判定
							bool isallzero = true;
							for (uint16_t i = 0; i < packet_frame->pointnum; i++)
							{
								if (packet_frame->pointdata[i].distance > 0 && packet_frame->pointdata[i].distance < 65533)
								{
									isallzero = false;
									break;
								}
							}
							if (isallzero)
							{
								sprintf(log_buf, "one frame is all zero");
								WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
							}
							WritetPointCloudCallBack(cfg->ID, 0, packet_frame, max_frame_len);
							points.clear();
							memset(packet_frame, 0, sizeof(Protocol::Packet_ZM));
							last_frame_timestamp = SystemAPI::GetTimeStamp(true);
						}
						if (ret == -2)
						{
							sprintf(log_buf, "merge one frame ng");
							WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
							points.clear();
							memset(packet_frame, 0, sizeof(Protocol::Packet_ZM));
						}
						if (ret == 1)
						{
							last_span_timestamp = SystemAPI::GetTimeStamp(true);
						}
						if (points.size() > 3600)
							points.clear();

						isfind = true;
						consume += idx;

						// if(ret==1||ret==2)
						// 	printf("%s %d %d %d\n", __FUNCTION__, __LINE__,idx,consume);

						break;
					} // 报警包
					else if (recv_buf[idx + 0] == 0x53 && recv_buf[idx + 1] == 0x54 && recv_buf[idx + 6] == 0x45 && recv_buf[idx + 7] == 0x44)
					{
						// printf("%s %d\n", __FUNCTION__, __LINE__);
						int minlen = idx + sizeof(Protocol::Alarm_ZM);
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}
						Protocol::Alarm_ZM *alarm_zm = (Protocol::Alarm_ZM *)(recv_buf + idx);
						uint32_t crc = BaseAPI::stm32crc_8((uint8_t *)alarm_zm, sizeof(Protocol::Alarm_ZM) - 4);
						consume = minlen;
						// MYLOG<<crc<<alarm_zm->crc;
						if (crc != alarm_zm->crc)
						{
							sprintf(log_buf, "alarm crc error");
							WriteLogDataCallBack(cfg->ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
							break;
						}
						// printf("%s %d %d %d\n", __FUNCTION__, __LINE__,idx,consume);
						WriteLogDataCallBack(cfg->ID, Protocol::MSG_ALARM, (char *)&alarm_zm->state, sizeof(uint32_t));
						isfind = true;
						break;
					} // 启停应答
					else if (recv_buf[idx + 0] == 0x0d && recv_buf[idx + 1] == 0x0a && recv_buf[idx + 2] == 0x4c && recv_buf[idx + 3] == 0x53 && recv_buf[idx + 4] == 0x54)
					{
						// printf("%s %d\n", __FUNCTION__, __LINE__);
						int minlen = idx + 32;
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}
						CmdTask &cmdtask = cmdtasklist.cmdtask.front();
						if (cmdtask.cmd == "LSTARH" || cmdtask.cmd == "LSTOPH")
							cmdtasklist.cmdtask.pop();
						std::string cmd_buf;
						std::string str = std::string((char *)&recv_buf[idx], 32);
						int separatorIdx = str.find("\r\n");
						std::string str2 = str.substr(separatorIdx + 2);
						int separatorIdx2 = str2.find("\r\n");

						int separatorIdx3 = str2.find(" ");
						std::string str3 = str2.substr(separatorIdx3 + 1, separatorIdx2 - separatorIdx3 - 1);

						int len = idx + separatorIdx + separatorIdx2 + 4;
						consume = len;

						cfg->action = Protocol::FINISH;
						cfg->recv_buf = str3;
						cfg->recv_len = str3.size();
						// printf("len:%d %d\n", separatorIdx, separatorIdx2);
						// printf("%s:%d %d  %s %d \n",cmdtask.cmd.c_str(), len, idx, str3.c_str(),str3.size());
						isfind = true;
						break;
					} // 设置转速
					else if (recv_buf[idx + 0] == 0x0d && recv_buf[idx + 1] == 0x0a && recv_buf[idx + 2] == 0x4c && recv_buf[idx + 3] == 0x53 && recv_buf[idx + 4] == 0x52)
					{
						// printf("%s %d\n", __FUNCTION__, __LINE__);
						int minlen = idx + 32;
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}
						CmdTask &cmdtask = cmdtasklist.cmdtask.front();
						if (cmdtask.cmd.find("LSRPM") != std::string::npos)
							cmdtasklist.cmdtask.pop();
						std::string cmd_buf;
						std::string str = std::string((char *)&recv_buf[idx], 32);
						int separatorIdx = str.find("\r\n");
						std::string str2 = str.substr(separatorIdx + 2);
						int separatorIdx2 = str2.find("\r\n");

						int separatorIdx3 = str2.find(" ");
						std::string str3 = str2.substr(separatorIdx3 + 1, separatorIdx2 - separatorIdx3 - 1);

						int len = idx + separatorIdx + separatorIdx2 + 4;
						consume = len;

						cfg->action = Protocol::FINISH;
						cfg->recv_buf = str3;
						cfg->recv_len = str3.size();
						// printf("len:%d %d\n", separatorIdx,separatorIdx2);
						// printf("len:%d %d  %s %d\n", len, idx, str3.c_str(),str3.size());
						isfind = true;
						break;
					} // 查询版本信息
					else if (recv_buf[idx + 0] == 0x0d && recv_buf[idx + 1] == 0x0a && recv_buf[idx + 2] == 0x4d && recv_buf[idx + 3] == 0x43 && recv_buf[idx + 4] == 0x55)
					{
						int minlen = idx + 128;
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}

						CmdTask &cmdtask = cmdtasklist.cmdtask.front();
						if (cmdtask.cmd == "LXVERH")
							cmdtasklist.cmdtask.pop();

						std::string mcuversion, motorversion, motorhversion;
						std::string str = std::string((char *)&recv_buf[idx], 128);
						int separatorIdx = str.find("\r\n");
						std::string str2 = str.substr(separatorIdx + 2);
						int separatorIdx2 = str2.find("\r\n");
						std::string str3 = str2.substr(separatorIdx2 + 2);
						int separatorIdx3 = str3.find("\r\n");
						std::string str4 = str3.substr(separatorIdx3 + 2);
						int separatorIdx4 = str4.find("\r\n");
						std::string str5 = str4.substr(separatorIdx4 + 2);
						int separatorIdx5 = str5.find("\r\n");
						std::string str6 = str5.substr(separatorIdx5 + 2);
						int separatorIdx6 = str6.find("\r\n");
						int len = idx + separatorIdx + separatorIdx2 + separatorIdx3 + separatorIdx4 + separatorIdx5 + separatorIdx6 + 12;
						consume = len;

						int mcu_idx = str2.find(":");
						if (separatorIdx2 != mcu_idx + 1)
							mcuversion = str2.substr(mcu_idx + 1, separatorIdx2 - mcu_idx - 1);

						int motor_idx = str3.find(":");
						if (separatorIdx3 != motor_idx + 1)
							motorversion = str3.substr(motor_idx + 1, separatorIdx3 - motor_idx - 1);

						int motor_h_idx = str4.find(":");
						if (separatorIdx4 != motor_idx + 1)
							motorhversion = str4.substr(motor_h_idx + 1, separatorIdx4 - motor_h_idx - 1);

						cfg->action = Protocol::FINISH;
						cfg->recv_buf = mcuversion + "|" + motorversion + "|" + motorhversion;
						cfg->recv_len = cfg->recv_buf.size();
						isfind = true;
						break;
					} // 查询SN
					else if (recv_buf[idx + 0] == 0x0d && recv_buf[idx + 1] == 0x0a && recv_buf[idx + 2] == 0x50 && recv_buf[idx + 3] == 0x52 && recv_buf[idx + 11] == 0x4e && recv_buf[idx + 12] == 0x3a)
					{
						int minlen = idx + 64;
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}
						CmdTask &cmdtask = cmdtasklist.cmdtask.front();
						if (cmdtask.cmd == "LUUIDH")
							cmdtasklist.cmdtask.pop();

						std::string sn;
						std::string str = std::string((char *)&recv_buf[idx], 64);
						int separatorIdx = str.find("\r\n");
						std::string str2 = str.substr(separatorIdx + 2);
						int separatorIdx2 = str2.find("\r\n");
						std::string str3 = str2.substr(separatorIdx2 + 2);
						int separatorIdx3 = str3.find("\r\n");
						std::string str4 = str3.substr(separatorIdx3 + 2);
						int separatorIdx4 = str4.find("\r\n");

						int sn_idx = str2.find(":");
						if (separatorIdx2 != sn_idx + 1)
							sn = str2.substr(sn_idx + 1, separatorIdx2 - sn_idx - 1);

						int len = idx + separatorIdx + separatorIdx2 + separatorIdx3 + separatorIdx4 + 8;
						consume = len;
						// printf("LUUIDH:%d %d  %s %u %u %llu\n", len, idx, sn.c_str(),sn.size(),cmdtasklist.cmdtask.size(),SystemAPI::GetTimeStamp(true));
						cfg->action = Protocol::FINISH;
						cfg->recv_buf = sn;
						cfg->recv_len = sn.size();
						isfind = true;
						break;
					}
				}
				// 如果两次解析出的长度一样，说明已经无法解析了
				// printf("%d %d %d \n",__LINE__,last_consume,consume);
				if (last_consume == consume)
					break;
				last_consume = consume;
			}
			// 如果缓存大于一半，并且找不到存在的包，则清空BUF_SIZE/4的缓存
			if (buf_len > BUF_SIZE / 2 && !isfind)
			{
				consume = BUF_SIZE / 4;
				sprintf(log_buf, "cache clear 1/4");
				WriteLogDataCallBack(cfg->ID, Protocol::MSG_DEBUG, log_buf, strlen(log_buf));
				// printf("%s %d no find %d\n", __FUNCTION__, __LINE__,buf_len);
			}
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!isfind)
				{
					bool isA5head=true;
					if(consume==5)
					{
						for(int i=0;i<consume;i++)
						{
							if(recv_buf[i]!=0xa5)
							{
								isA5head = false;
								break;
							}
						}
					}
					else
					{
						isA5head=false;
					}

					if(!isA5head)
						printf("drop %d bytes: %02x %02x %02x %02x %02x %02x \n",
						   consume,
						   recv_buf[0], recv_buf[1], recv_buf[2],
						   recv_buf[3], recv_buf[4], recv_buf[5]);
				}

				for (int i = consume; i < buf_len; i++)
					recv_buf[i - consume] = recv_buf[i];

				// printf("%s %d %d %d\n", __FUNCTION__, __LINE__,buf_len,consume);
				buf_len -= consume;
			}
		}
	}
	delete[] recv_buf;
}

int PaceCatLidarSDK::parsePointCloud(int ID, uint8_t *data, uint16_t len, int32_t &sequence_num, uint64_t &last_timestamp, uint16_t &last_start_angle, int &first_start_angle, uint16_t &consume, std::vector<Protocol::Point_ZM> &points, Protocol::Packet_ZM &packet_frame)
{
	Protocol::Packet_Head_ZM *head = (Protocol::Packet_Head_ZM *)data;
	int minlen = sizeof(Protocol::Packet_Head_ZM);
	if (minlen > len)
		return 0;

	int point_num = 0;
	if (head->data_type == 0)
		point_num = 180;
	else if (head->data_type == 1)
		point_num = 120;
	else if (head->data_type == 2)
		point_num = 90;

	minlen = sizeof(Protocol::Packet_Head_ZM) + sizeof(Protocol::Packet_Body_ZM) + sizeof(Protocol::Point_ZM) * point_num;
	if (minlen > len)
		return 0;
	consume = minlen;
	char log_buf[1024] = {0};
	int result = 0;
	Protocol::Point_ZM *point_zm = (Protocol::Point_ZM *)(&head->data[0]);
	Protocol::Packet_Body_ZM *packet_body_zm = (Protocol::Packet_Body_ZM *)(&head->data[0] + sizeof(Protocol::Point_ZM) * point_num);
	int packet_idx_diff = (int)(packet_body_zm->sequence_num) - (int)sequence_num;
	uint64_t packet_timestamp = datetime_to_nanoseconds(head->date_time[0] + 1900, head->date_time[1], head->date_time[2],
														head->date_time[3], head->date_time[4], head->date_time[5], head->timestamp * 1000);
	int64_t timestamp_diff = packet_timestamp - last_timestamp;

	if (packet_idx_diff != 1 && packet_idx_diff != -65535 && sequence_num >= 0)
	{
		sprintf(log_buf, "drop packet last idx:%d current idx:%d last start angle:%d current start angle:%d last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, last_start_angle, head->start_angle / 100, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
		result = -3;
	}
	if (timestamp_diff > 10000000 && last_timestamp)
	{
		sprintf(log_buf, "time interval large,packet last idx:%d current idx:%d diff:%ld,last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, timestamp_diff, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
	}
	else if (timestamp_diff < 0 && last_timestamp)
	{
		sprintf(log_buf, "time jumpback,packet last idx:%d current idx:%d diff:%ld,last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, timestamp_diff, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, Protocol::MSG_ERROR, log_buf, strlen(log_buf));
	}

	if (packet_idx_diff == 1 || packet_idx_diff == -65535)
	{
		for (uint16_t idx = 0; idx < point_num; idx++)
		{
			points.push_back(point_zm[idx]);
		}
		result = 1;
		// 扇区完整性判定:当接收到最后一个扇区，即起始角度为342度的时候，检查点数量
		int end_angle = ((first_start_angle + 360) - 18) % 360;
		if (head->start_angle / 100 == 90 - 18)
		{
			if ((uint32_t)(point_num * 20) == points.size())
			{
				for (uint16_t i = 0; i < points.size(); i++)
				{
					packet_frame.pointdata[i] = points[i];
				}
				packet_frame.pointnum = points.size();
				packet_frame.ts_end[0] = packet_timestamp / 1000000000;
				packet_frame.ts_end[1] = packet_timestamp % 1000000000;

				result = 2;
			}
			else
			{
				// 不完整数据 清空帧
				result = -2;
			}
		}
		if (head->start_angle / 100 == 90)
		{
			packet_frame.ts_beg[0] = packet_timestamp / 1000000000;
			packet_frame.ts_beg[1] = packet_timestamp % 1000000000;
		}
	}
	sequence_num = packet_body_zm->sequence_num;
	last_timestamp = packet_timestamp;
	last_start_angle = head->start_angle / 100;
	if (first_start_angle == -1)
		first_start_angle = last_start_angle;
	return result;
}
bool uart_talk(int fd, int n, const char *cmd, int nhdr, const char *hdr_str, int nfetch, char *fetch, int waittime)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);

	char buf[4096];
	int nr = read(fd, buf, sizeof(buf));
	int idx = waittime;
	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf + nr, sizeof(buf) - nr);
		// printf(" fd %d %d \n",n,nr);
		if (n > 0)
		{
			nr += n;
			idx = waittime;
		}
		else if (n == 0)
		{
			idx--;
			usleep(1000);
			if (idx == 0)
			{
				// printf("read 0 byte max index break\n");
				break;
			}
		}
	}
	// if(idx>0)
	//     printf("read max byte break\n");

	for (unsigned int i = 0; i < sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0 && nhdr > 0)
		{
			if (nfetch > 0)
			{
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0 || strcmp(cmd, "LQAZNH") == 0 || strcmp(cmd, "LQPSTH") == 0 || strcmp(cmd, "LQNPNH") == 0 || strcmp(cmd, "LQOUTH") == 0 || strcmp(cmd, "LQCIRH") == 0 || strcmp(cmd, "LQFIRH") == 0 || strcmp(cmd, "LQSRPMH") == 0 || strcmp(cmd, "LQSMTH") == 0 || strcmp(cmd, "LQDSWH") == 0 || strcmp(cmd, "LQZTPH") == 0 || strcmp(cmd, "LQSAFH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else if (strstr(cmd, "LSRPM") != NULL)
				{
					if (buf[i + nhdr + 1] == 'O' && buf[i + nhdr + 2] == 'K')
					{
						strncpy(fetch, "OK", 3);
						fetch[3] = 0;
					}
					else if (buf[i + nhdr + 1] == 'e' && buf[i + nhdr + 2] == 'r')
					{
						strncpy(fetch, "NG", 3);
						fetch[3] = 0;
					}
				}
				else
				{
					strncpy(fetch, "OK", 3);
					fetch[3] = 0;
				}
			}
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				if (buf[i + n + 1] == 'E' && buf[i + n + 2] == 'R')
				{
					fetch[0] = 'N';
					fetch[1] = 'G';
				}
				fetch[2] = 0;
			}
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport");
				fetch[10] = 0;
			}
			return true;
		}
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

uint8_t *PaceCatLidarSDK::load_bin(const char *path, size_t &len)
{
	FILE *fp;
	if ((fp = fopen(path, "rb")) == NULL)
	{
		return NULL;
	}
	uint8_t *file_buf = new uint8_t[1024 * 1024];
	len = fread(file_buf, 1, 1024 * 1024, fp);
	fclose(fp);

	return file_buf;
}
int PaceCatLidarSDK::readbuf(int fd, uint8_t min_len, uint16_t max_len, uint16_t timeout, uint8_t max_try_time, uint8_t function_code, unsigned char *send_buf, int send_len, unsigned char *recv_buf, int &recv_len)
{
	int cmd_len = 0;
	unsigned char cmd_buf[MAX_CMD_BUFFER] = {0};
	for (int i = max_try_time; i >= 0; i--)
	{
		uint64_t cmd_timestamp = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
		cmd_len = 0;
		memset(cmd_buf, 0, MAX_CMD_BUFFER);

		// 发送A5前导码
		unsigned char preamble[5] = {0xA5, 0xA5, 0XA5, 0XA5, 0XA5};
		write(fd, preamble, 5);
		// 发送具体的指令
		write(fd, send_buf, send_len);
		// MYLOG<<"write"<<i;

		bool isok = false;
		while (cmd_len < max_len)
		{
			uint64_t tmp_timestamp = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
			if (tmp_timestamp - cmd_timestamp > timeout)
				break;
			int nr = read(fd, cmd_buf + cmd_len, max_len - cmd_len);
			if (nr > 0)
				cmd_len += nr;
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}

			if (cmd_len >= min_len + 5)
			{
				int idx = -1;
				for (int j = 0; j <= cmd_len - min_len; j++)
				{
					if (cmd_buf[j] == 0x4c && cmd_buf[j + 1] == 0x48)
					{
						idx = j;
						Protocol::RX_CmdHeader *rt_cmd = (Protocol::RX_CmdHeader *)(cmd_buf + idx);
						int need_len = idx + rt_cmd->size + 4;
						// MYLOG<<idx<<need_len<<cmd_len;
						if (need_len > cmd_len)
						{
							if (need_len > MAX_CMD_BUFFER)
							{
								cmd_len = MAX_CMD_BUFFER / 2;
								memset(&cmd_buf[0], 0, cmd_len);
								memcpy(&cmd_buf[0], &cmd_buf[cmd_len], cmd_len);
							}
						}
						else
						{
							isok = true;
						}
						break;
					}
				}
			}
			if (isok)
				break;
		}

		if (cmd_len < min_len + 5)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
			// MYLOG<<"outtime"<<cmd_len;
			continue;
		}

		// 开始搜索应答
		for (int j = 0; j <= cmd_len - min_len; j++)
		{
			if (cmd_buf[j] == 0x4c && cmd_buf[j + 1] == 0x48)
			{
				Protocol::RX_CmdHeader *rx_cmd = (Protocol::RX_CmdHeader *)(cmd_buf + j);
				// 判断测试的功能是否相同
				// 应答总长度
				recv_len = rx_cmd->size + 4;
				if (j + recv_len > cmd_len)
					return -1;

				uint32_t crc = BaseAPI::stm32crc_8((uint8_t *)rx_cmd, rx_cmd->size);
				uint32_t crc2;
				memcpy(&crc2, (uint8_t *)rx_cmd + rx_cmd->size, sizeof(uint32_t));
				// MYLOG<<crc<<crc2;
				if (crc != crc2)
					continue;
				// 返回的是同一个应答
				// MYLOG<<rx_cmd->cmd<<function_code<<rx_cmd->atk;
				if (rx_cmd->cmd == function_code)
				{
					// 如果是查询版本号
					if (function_code == Protocol::ZM_CMD_TYPE_SET_QUERY_VERSION)
					{
						recv_len = rx_cmd->size - 9;
						memcpy(recv_buf, rx_cmd->data, recv_len);
					}
					if (rx_cmd->atk == Protocol::ZM_ACK_TYPE_TIMEOUT)
					{
						continue;
					}
					if (rx_cmd->atk != Protocol::ZM_ACK_TYPE_OK)
					{
						// MYLOG<<rx_cmd->atk;
						break;
					}
					return rx_cmd->atk;
				}
			}
		}
	}
	return -1;
}

bool PaceCatLidarSDK::checkBPS_isok(int fd)
{
	int cmd_len = 0;
	unsigned char cmd_buf[1024] = {0};
	int max_len = 1024;
	uint64_t timestamp = SystemAPI::getTimestamp(TimeUnit::MILLISECOND) + 4000;

	//(12+5)*3
	while (cmd_len < 51)
	{
		int nr = read(fd, cmd_buf + cmd_len, max_len - cmd_len);
		if (nr > 0)
		{
			cmd_len += nr;
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

		uint64_t tmp_timestamp = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
		if (tmp_timestamp > timestamp)
			return false;
	}
	for (int i = 0; i < cmd_len - 17; i++)
	{
		if (cmd_buf[i] == 0x53 && cmd_buf[i + 1] == 0x54 && cmd_buf[i + 6] == 0x45 && cmd_buf[i + 7] == 0x44)
		{
			if (cmd_buf[i + 17] == 0x53 && cmd_buf[i + 1 + 17] == 0x54 && cmd_buf[i + 6 + 17] == 0x45 && cmd_buf[i + 7 + 17] == 0x44)
			{
				return true;
			}
		}
	}
	return false;
}
