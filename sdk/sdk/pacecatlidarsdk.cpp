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
		m_lidars.at(i)->run_state = ZhuiMiProtocol::QUIT;
	}
}
int PaceCatLidarSDK::AddLidar(std::string com_name, uint32_t bandrate)
{
	RunConfig *cfg = new RunConfig;
	cfg->comname = com_name;
	cfg->baudrate = bandrate;
	cfg->ID = m_lidar_id_counter++;
	cfg->run_state = ZhuiMiProtocol::ONLINE;
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

	lidar->thread_subData = std::thread(&PaceCatLidarSDK::UartThreadProc, PaceCatLidarSDK::getInstance(), ID);
	lidar->thread_subData.detach();
	return true;
}
bool PaceCatLidarSDK::DisconnectLidar(uint16_t ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != ZhuiMiProtocol::QUIT)
		{
			m_lidars.at(i)->run_state = ZhuiMiProtocol::OFFLINE;
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
	lidar->action = ZhuiMiProtocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != ZhuiMiProtocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == ZhuiMiProtocol::FINISH)
	{
		// printf("%s %d %s\n", __FUNCTION__, __LINE__,lidar->recv_buf.c_str());
		lidar->action = ZhuiMiProtocol::NONE;
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
	lidar->action = ZhuiMiProtocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != ZhuiMiProtocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == ZhuiMiProtocol::FINISH)
	{
		lidar->action = ZhuiMiProtocol::NONE;
		info = lidar->recv_buf;
		// printf("%s %d %s\n", __FUNCTION__, __LINE__,lidar->recv_buf.c_str());

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
	if (action == ZhuiMiProtocol::START)
		lidar->send_buf = "LSTARH";
	else if (action == ZhuiMiProtocol::STOP)
		lidar->send_buf = "LSTOPH";
	else
		return false;

	lidar->action = ZhuiMiProtocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != ZhuiMiProtocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == ZhuiMiProtocol::FINISH)
	{
		lidar->action = ZhuiMiProtocol::NONE;
		printf("%s %d %s\n", __FUNCTION__, __LINE__, lidar->recv_buf.c_str());

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
	lidar->action = ZhuiMiProtocol::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != ZhuiMiProtocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == ZhuiMiProtocol::FINISH)
	{
		lidar->action = ZhuiMiProtocol::NONE;
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
	sprintf(log_buf, "start upgrade:com_name:%s baudrate:%d path:%s", lidar->comname.c_str(), lidar->baudrate, path.c_str());
	WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_WARM, log_buf, strlen(log_buf));

	size_t len = 0;
	uint8_t *file = load_bin(path.c_str(), len);
	if (!file)
	{
		sprintf(log_buf, "load  file failed:%s", path.c_str());
		WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
		return false;
	}
	sprintf(log_buf, "load firmware file");
	WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_WARM, log_buf, strlen(log_buf));

	int h = SystemAPI::open_serial_port(lidar->comname.c_str(), lidar->baudrate);
	if (h <= 0)
	{
		sprintf(log_buf, "can not open:%s %d", lidar->comname.c_str(), lidar->baudrate);
		WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
		return false;
	}
	char line[1025] = {0};
	unsigned long dw;

	for (int i = 5; i >= 0; i--)
	{
		write(h, "LSBPS:500000H", 13);
		// write(h, "LSTOPH", 6);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	change_baud(h, 500000);

	uint64_t t = SystemAPI::GetTimeStamp(true);
	uint64_t t2 = t;
	bool bReady = false;
	int trynum = 5;
	while (!bReady)
	{
		if (SystemAPI::GetTimeStamp(true) > t + 500)
		{
			dw = write(h, "LFWUPH", 6);
			t = SystemAPI::GetTimeStamp(true);
		}
		if (SystemAPI::GetTimeStamp(true) > t + 1500)
		{
			t = SystemAPI::GetTimeStamp(true);
			if (trynum > 0)
			{
				write(h, "LFWUPH", 6);
				trynum--;
			}
			else
			{
				SystemAPI::closefd(h, false);
				sprintf(log_buf, "LFWUPH talk error");
				WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
				return false;
			}
		}
		dw = XReadFile(h, line, 1024);
		if (dw > 0)
		{
			line[dw] = 0;
			// printf("lidar : %s\n", line);
			for (unsigned int i = 0; i + 6 <= dw; i++)
			{
				if (memcmp(line + i, "LFWUQH", 6) == 0)
				{
					bReady = true;
					break;
				}
			}
		}
	}
	int nl = sprintf(line, "LFWUI:%08lX,%08X,H", len, 0xffffffff);
	// MYLOG<<QByteArray(line,25);
	write(h, line, nl);
	bReady = false;
	int nr = 0;
	while (!bReady && nr < 1000)
	{
		int r = XReadFile(h, line + nr, 1024 - nr);
		if (r > 0)
		{
			line[nr + r - 1] = 0;
			// printf("lidar : %s\n", line+nr);
			nr += r;
			for (int i = 0; i + 7 <= nr; i++)
			{
				if (memcmp(line + i, "FWUI OK", 7) == 0)
				{
					bReady = true;
					break;
				}
			}
		}
		if (SystemAPI::GetTimeStamp(true) > t + 2000)
		{
			break;
		}
	}
	if (!bReady)
	{
		sprintf(log_buf, "send bin length failed");
		WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
		SystemAPI::closefd(h, false);
		return false;
	}
	sprintf(log_buf, "send bin length ok");
	WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_WARM, log_buf, strlen(log_buf));

	if (xmodem_send(h, len, file) != 0)
	{
		sprintf(log_buf, "transfer file error");
		WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
		SystemAPI::closefd(h, false);
		return false;
	}
	sprintf(log_buf, "firmware file update success");
	WriteLogDataCallBack(lidar->ID, ZhuiMiProtocol::MSG_WARM, log_buf, strlen(log_buf));
	SystemAPI::closefd(h, false);
	return true;
}

bool PaceCatLidarSDK::ClearFrameCache(uint16_t ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->action = ZhuiMiProtocol::CACHE_CLEAR;
	int index = CMD_REPEAT;
	while (lidar->action != ZhuiMiProtocol::FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == ZhuiMiProtocol::FINISH)
	{
		lidar->action = ZhuiMiProtocol::NONE;
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
	uint16_t max_frame_len = sizeof(ZhuiMiProtocol::Packet_ZM) + sizeof(ZhuiMiProtocol::Point_ZM) * 3600;
	ZhuiMiProtocol::Packet_ZM *packet_frame = (ZhuiMiProtocol::Packet_ZM *)malloc(max_frame_len);

	char log_buf[1024] = {0};
	int buf_len = 0;
	sprintf(log_buf, "wait lidar data:com_name:%s baudrate:%d ...", cfg->comname.c_str(), cfg->baudrate);
	WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
	std::vector<ZhuiMiProtocol::Point_ZM> points;
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

	while (cfg->run_state != ZhuiMiProtocol::QUIT)
	{
		// 任务分发
		if (cfg->action == ZhuiMiProtocol::CMD_TALK)
		{
			// printf("%s %d\n", __FUNCTION__, __LINE__);
			cmdtasklist.cmdtask.push(CmdTask{0, 0, cfg->send_buf.c_str()});
			cfg->action = ZhuiMiProtocol::NONE;
		}
		else if (cfg->action == ZhuiMiProtocol::CACHE_CLEAR)
		{
			points.clear();
			sequence_num = 0;
			memset(packet_frame, 0, sizeof(ZhuiMiProtocol::Packet_ZM));
			cfg->action = ZhuiMiProtocol::FINISH;
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
				WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));

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
			WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
			last_frame_timestamp = timestamp;
		}
		if (last_span_timestamp != 0 && timestamp - last_span_timestamp > 1000)
		{
			sprintf(log_buf, "1s no span data");
			WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
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
			WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
			return;
		}
		else if (ret == 0)
		{
			sprintf(log_buf, "no data");
			WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		if (cfg->handle > 0 && FD_ISSET(cfg->handle, &fds))
		{
			int nr = read(cfg->handle, recv_buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				// sprintf(log_buf, "cache len: %d  read len: %d", buf_len, nr);
				// WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
				// break;
				continue;
			}
			if (nr == 0)
			{
				continue;
			}
			if (nr > 0)
			{
				//printf("%d %d\n",buf_len,nr);
				buf_len += nr;
				if (buf_len >= (BUF_SIZE / 2))
				{
					sprintf(log_buf, "cache is too many:%d %d", buf_len, nr);
					WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
				}
			}
		}
		if (buf_len > 0)
		{
			uint16_t consume = 0;
			bool isfind = false;
			uint16_t last_consume=0;
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
						if(ret==0)
						{
							consume=idx;
							break;
						}
						if (ret == 2)
						{
							if (is_first_frame)
							{
								is_first_frame = false;
								sprintf(log_buf, "PaceCat sdk start work");
								WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
							}
							// printf("%s %d %d %d\n", __FUNCTION__, __LINE__,points.size(),consume);
							packet_frame->start_angle = first_start_angle;
							//帧数据全为0的判定
							bool isallzero=true;
							for(uint16_t i=0;i<packet_frame->pointnum;i++)
							{
								if(packet_frame->pointdata[i].distance>0&&packet_frame->pointdata[i].distance<65533)
								{
									isallzero=false;
									break;
								}
							}
							if(isallzero)
							{
								sprintf(log_buf, "one frame is all zero");
								WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
							}
							WritetPointCloudCallBack(cfg->ID, 0, packet_frame, max_frame_len);
							points.clear();
							memset(packet_frame, 0, sizeof(ZhuiMiProtocol::Packet_ZM));
							last_frame_timestamp = SystemAPI::GetTimeStamp(true);
						}
						if (ret == -2)
						{
							sprintf(log_buf, "merge one frame ng");
							WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
							points.clear();
							memset(packet_frame, 0, sizeof(ZhuiMiProtocol::Packet_ZM));
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
						int minlen = idx + sizeof(ZhuiMiProtocol::Alarm_ZM);
						if (minlen > buf_len)
						{
							consume = idx;
							break;
						}
						ZhuiMiProtocol::Alarm_ZM *alarm_zm = (ZhuiMiProtocol::Alarm_ZM *)(recv_buf + idx);
						uint32_t crc = BaseAPI::stm32crc_8((uint8_t *)alarm_zm, sizeof(ZhuiMiProtocol::Alarm_ZM) - 4);
						consume = minlen;
						// MYLOG<<crc<<alarm_zm->crc;
						if (crc != alarm_zm->crc)
						{
							sprintf(log_buf, "alarm crc error");
							WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
							break;
						}
						//printf("%s %d %d %d\n", __FUNCTION__, __LINE__,idx,consume);
						WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_ALARM, (char *)&alarm_zm->state, sizeof(uint32_t));
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

						cfg->action = ZhuiMiProtocol::FINISH;
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

						cfg->action = ZhuiMiProtocol::FINISH;
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
						int len = idx+separatorIdx + separatorIdx2 + separatorIdx3 + separatorIdx4 + separatorIdx5 + separatorIdx6 + 12;
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

						cfg->action = ZhuiMiProtocol::FINISH;
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
						cfg->action = ZhuiMiProtocol::FINISH;
						cfg->recv_buf = sn;
						cfg->recv_len = sn.size();
						isfind = true;
						break;
					}
				}
				//如果两次解析出的长度一样，说明已经无法解析了
				//printf("%d %d %d \n",__LINE__,last_consume,consume);
				if(last_consume == consume)
					break;
				last_consume =consume;
			}
			// 如果缓存大于一半，并且找不到存在的包，则清空BUF_SIZE/4的缓存
			if (buf_len > BUF_SIZE / 2 && !isfind)
			{
				consume = BUF_SIZE / 4;
				sprintf(log_buf, "cache clear 1/4");
				WriteLogDataCallBack(cfg->ID, ZhuiMiProtocol::MSG_DEBUG, log_buf, strlen(log_buf));
				// printf("%s %d no find %d\n", __FUNCTION__, __LINE__,buf_len);
			}
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!isfind)
				{
					printf("drop %d bytes: %02x %02x %02x %02x %02x %02x \n",
						   consume,
						   recv_buf[0], recv_buf[1], recv_buf[2],
						   recv_buf[3], recv_buf[4], recv_buf[5]);
				}

				for (int i = consume; i < buf_len; i++)
					recv_buf[i - consume] = recv_buf[i];

				//printf("%s %d %d %d\n", __FUNCTION__, __LINE__,buf_len,consume);
				buf_len -= consume;
			}
		}
	}
	delete[] recv_buf;
}

int PaceCatLidarSDK::parsePointCloud(int ID, uint8_t *data, uint16_t len, int32_t &sequence_num, uint64_t &last_timestamp, uint16_t &last_start_angle, int &first_start_angle, uint16_t &consume, std::vector<ZhuiMiProtocol::Point_ZM> &points, ZhuiMiProtocol::Packet_ZM &packet_frame)
{
	ZhuiMiProtocol::Packet_Head_ZM *head = (ZhuiMiProtocol::Packet_Head_ZM *)data;
	int minlen = sizeof(ZhuiMiProtocol::Packet_Head_ZM);
	if (minlen > len)
		return 0;

	int point_num = 0;
	if (head->data_type == 0)
		point_num = 180;
	else if (head->data_type == 1)
		point_num = 120;
	else if (head->data_type == 2)
		point_num = 90;

	minlen = sizeof(ZhuiMiProtocol::Packet_Head_ZM) + sizeof(ZhuiMiProtocol::Packet_Body_ZM) + sizeof(ZhuiMiProtocol::Point_ZM) * point_num;
	if (minlen > len)
		return 0;
	consume = minlen;
	char log_buf[1024] = {0};
	int result = 0;
	ZhuiMiProtocol::Point_ZM *point_zm = (ZhuiMiProtocol::Point_ZM *)(&head->data[0]);
	ZhuiMiProtocol::Packet_Body_ZM *packet_body_zm = (ZhuiMiProtocol::Packet_Body_ZM *)(&head->data[0] + sizeof(ZhuiMiProtocol::Point_ZM) * point_num);
	int packet_idx_diff = (int)(packet_body_zm->sequence_num) - (int)sequence_num;
	uint64_t packet_timestamp = datetime_to_nanoseconds(head->date_time[0] + 1900, head->date_time[1], head->date_time[2],
														head->date_time[3], head->date_time[4], head->date_time[5], head->timestamp * 1000);
	int64_t timestamp_diff = packet_timestamp - last_timestamp;

	if (packet_idx_diff != 1 && packet_idx_diff != -65535 && sequence_num >= 0)
	{
		sprintf(log_buf, "drop packet last idx:%d current idx:%d last start angle:%d current start angle:%d last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, last_start_angle, head->start_angle / 100, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
		result = -3;
	}
	if (timestamp_diff > 10000000 && last_timestamp)
	{
		sprintf(log_buf, "time interval large,packet last idx:%d current idx:%d diff:%ld,last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, timestamp_diff, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
	}
	else if (timestamp_diff < 0 && last_timestamp)
	{
		sprintf(log_buf, "time jumpback,packet last idx:%d current idx:%d diff:%ld,last time:%ld current time:%ld", sequence_num, packet_body_zm->sequence_num, timestamp_diff, last_timestamp, packet_timestamp);
		WriteLogDataCallBack(ID, ZhuiMiProtocol::MSG_ERROR, log_buf, strlen(log_buf));
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
		if (head->start_angle / 100 == end_angle)
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
		if (head->start_angle / 100 == first_start_angle)
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

int PaceCatLidarSDK::XReadFile(int fd, char *line, int sz)
{
	int nr = 0;
	uint64_t t = SystemAPI::GetTimeStamp(true);
	int zeroNum = 3;
	while (SystemAPI::GetTimeStamp(true) < t + 200 && nr < sz)
	{
		int dw = read(fd, line + nr, sz - nr);
		if (dw > 0)
		{
			nr += dw;
		}
		else if (dw == 0)
		{
			zeroNum--;
			if (zeroNum == 0)
				break;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	return nr;
}
int PaceCatLidarSDK::xymodem_send(int serial_fd, size_t len, const uint8_t *buf, int protocol, int wait)
{
	int ret;
	uint8_t answer;
	uint8_t eof = X_EOF;
	ZhuiMiProtocol::xmodem_chunk chunk;
	int skip_payload = 0;

	uint64_t t = SystemAPI::GetTimeStamp(true);

	if (wait)
	{
		printf("wait for lidar transfer signal ...\n");
		// fflush(stdout);

		do
		{
			if (SystemAPI::GetTimeStamp(true) > t + 5000)
			{
				return -1;
			}
			ret = xread(serial_fd, &answer, 1);
			if (ret != sizeof(answer))
			{
				// printf("read error\n");
				continue;
				// return -1;// errno;
			}
		} while (answer != 'C');
		// printf("done.\n");
	}

	printf("Sending ");

	if (protocol == PROTOCOL_YMODEM)
	{
		// strncpy((char*)chunk.payload, "filename", sizeof(chunk.payload));
		chunk.block = 0;
		skip_payload = 1;
	}
	else
	{
		chunk.block = 1;
	}

	chunk.start = X_STX;

	while (len)
	{
		size_t z = 0;
		int next = 0;
		char status;

		if (!skip_payload)
		{
			z = CMP_MIN(len, sizeof(chunk.payload));
			memcpy(chunk.payload, buf, z);
			memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);
		}
		else
		{
			skip_payload = 0;
		}

		// chunk.crc = swap16(crc16(chunk.payload, sizeof(chunk.payload)));
		chunk.crc = swap16(calcrc(chunk.payload, sizeof(chunk.payload)));
		chunk.block_neg = 0xff - chunk.block;
		ret = xwrite(serial_fd, &chunk, sizeof(chunk));
		if (ret != sizeof(chunk))
			return -2; // errno;
		// ret = read_char(serial_fd, &answer, 3000);
		ret = xread(serial_fd, &answer, 1);
		if (ret != sizeof(answer))
		{
			return -3; // errno;
		}
		switch (answer)
		{
		case X_NAK:
			status = 'N';
			break;
		case X_ACK:
			status = '.';
			next = 1;
			break;
		default:
			status = '?';
			break;
		}

		printf("%c", status);
		fflush(stdout);

		if (next)
		{
			chunk.block++;
			len -= z;
			buf += z;
		}
	}
	ret = xwrite(serial_fd, &eof, sizeof(eof));
	if (ret != sizeof(eof))
		return -4; // errno;
	/* send EOT again for YMODEM */
	if (protocol == PROTOCOL_YMODEM)
	{
		ret = xwrite(serial_fd, &eof, sizeof(eof));
		if (ret != sizeof(eof))
			return -5; // errno;
	}

	printf("transfer bin finished\n");

	return 0;
}

int PaceCatLidarSDK::xmodem_send(int serial_fd, size_t len, const uint8_t *buf)
{
	return xymodem_send(serial_fd, len, buf, PROTOCOL_XMODEM, 1);
}
int PaceCatLidarSDK::xread(int fd, void *buf, size_t sz)
{
	size_t nr = 0;
	int idx = UART_WAITTIME;
	while (nr < sz)
	{
		int dw = read(fd, buf, sz - nr);
		if (dw > 0)
		{
			nr += dw;
			idx = UART_WAITTIME;
		}
		else if (dw <= 0)
		{
			idx--;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			if (idx == 0)
				break;
		}
		else
			break;
	}
	return nr;
}

int PaceCatLidarSDK::xwrite(int fd, void *buf, size_t sz)
{
	return write(fd, buf, sz);
}
