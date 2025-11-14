#ifndef E210_D_PROTOCOL_H
#define E210_D_PROTOCOL_H

#include "define.h"

#pragma pack(push, 1)
namespace ZhuiMiProtocol
{
	
struct Point_ZM
{
    uint16_t distance;//1mm
    uint8_t reflectivity;
};
struct Packet_Head_ZM {
    uint16_t code; // 0xEEFF
    uint8_t ver_maj;
    uint8_t ver_min;
    uint8_t ver_debug;
    uint8_t data_type;//数据类型   0/1/2    180/120/90点数   600/900/1200转
    uint8_t date_time[6];//年月日时分秒   年-1900
    uint32_t timestamp;//秒后小数部分   us   0-999999
    uint16_t start_angle;//起始角度   单位:0.01
    uint8_t data[0];
};
//Point_ZM point_data[180];  //18度一个扇区120个点
struct Packet_Body_ZM {
    uint32_t dirty_degree;//未使用
    uint8_t lidar_state;//未使用
    uint8_t soft_version;
    uint16_t temperature;//温度   单位0.1
    uint16_t sequence_num;//包序号
    uint32_t crc;
};
struct Alarm_ZM
{
    uint8_t sign_start[2];
    uint32_t state;
    uint8_t sign_end[2];
    uint32_t crc;
};

// struct Packet_ZM
// {
//     uint16_t pointnum;
//     uint8_t  date_time[6];
//     uint32_t beg_timestamp;
//     Point_ZM pointdata[0];
// };
struct Packet_ZM
{

    uint16_t start_angle;
    uint16_t pointnum;
    uint32_t ts_beg[2];
	uint32_t ts_end[2];
    Point_ZM pointdata[0];
};


struct FirmwareFile
{
	int code;
	int len;
	int sent;
	uint32_t crc;
	uint8_t date[4];
	uint8_t unused[120];
	char describe[512];
	uint8_t buffer[0];
};

struct FirmwarePart
{
	uint32_t offset;
	uint32_t crc;
	uint32_t buf[128];
};

struct FirmWriteResp
{
	uint32_t offset;
	int result;
	char msg[128];
};
struct ResendPack
{
	time_t timeout;
	uint32_t tried;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
	char buf[2048];
};
struct xmodem_chunk {
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[128];
    uint16_t crc;
};
struct UartState
{
    //byte1
    uint8_t unit_mm:1;//0 cm 1 mm
    uint8_t with_conf:1;//0 close 1 open
    uint8_t with_fitter:1;//去拖点
    uint8_t with_smooth:1;//滤波
    uint8_t span_9:1;
    uint8_t span_18:1;
    uint8_t span_other:1;
    uint8_t resampele:1;//重采样
    //byte2

    uint8_t moter_turn:1;//0正向 1反向
    uint8_t span_8:1;
    uint8_t span_16:1;
    uint8_t reserve2:5;

    //byte3
    uint8_t coil_disconnection:1;//线圈断开
    uint8_t overcurrent:1;//线圈过流
    uint8_t encoder_error:1;//码盘错误
    uint8_t motor_fault:1;//电机故障
    uint8_t low_time_synchronization_accuracy:1;//时间同步精度低
    uint8_t no_gps_signal:1;//没有GPS信号
    uint8_t no_pps_signal:1;//没有PPS信号
    uint8_t gps_signal_abnormal:1;//GPS信号异常
    //byte4
    uint8_t backplane_low_voltage:1;//底板低压
    uint8_t backplane_high_voltage:1;//底板高压
    uint8_t head_temperature_abnormal:1;//机头温度异常
    uint8_t head_low_loltage:1;//机头低压
    uint8_t head_high_loltage:1;//机头高压
    uint8_t head_no_data:1;//机头无数据
    uint8_t head_data_abnormal:1;//机头数据异常
    uint8_t infrared_receive_error:1;//红外接收错误
};
enum LidarState
{
	OFFLINE = 0,
	ONLINE,
	QUIT
};
enum LidarAction
{
	NONE,
	FINISH,
	START,
	STOP,
	CMD_TALK,
	// GET_SN,
	// GET_VERSION,
	// UPGRADE,
	CACHE_CLEAR,
};
enum PrintDevel
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR,
	MSG_ALARM
};
}





#pragma pack(pop)

typedef void (*LidarCloudPointCallback)(uint16_t handle, const uint8_t msg_type, const void *data, uint16_t data_len);
typedef void (*LidarLogDataCallback)(uint16_t handle, const uint8_t msg_type, const char *data, uint16_t data_len);

#endif
