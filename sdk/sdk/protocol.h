#ifndef E210_D_PROTOCOL_H
#define E210_D_PROTOCOL_H

#include "define.h"

#pragma pack(push, 1)
namespace Protocol
{




    
typedef enum {
 ZM_CMD_TYPE_START = 0, // 0x00: 启动雷达
 ZM_CMD_TYPE_STOP, // 0x01: 停⽌雷达
 ZM_CMD_TYPE_SET_BPS, // 0x02: 设置波特率，⽆返回
 ZM_CMD_TYPE_SET_RPM, // 0x03: 设置转速
 ZM_CMD_TYPE_SET_QUERY_VERSION, // 0x04: 查询固件版本号
 ZM_CMD_TYPE_START_UPDATE, // 0x05: 固件升级开始
 ZM_CMD_TYPE_FIRMWARE_INFO, // 0x06: 传输固件信息
 ZM_CMD_TYPE_FIRMWARE_DATA, // 0x07: 传输固件数据
 ZM_CMD_TYPE_END_UPDATE // 0x08: 查询固件升级状态
 } ZM_POTOCOL_CMD_TYPE_EN;

typedef enum {
 ZM_ACK_TYPE_OK = 0, // 0x00000000: 操作成功
 ZM_ACK_TYPE_ERROR, // 0x00000001: 通⽤错误
 ZM_ACK_TYPE_TIMEOUT, // 0x00000002: 超时
 ZM_ACK_TYPE_FILE_SIZE_ERR, // 0x00000003: ⽂件⼤⼩错误
 ZM_ACK_TYPE_FILE_SIZE_CHECK_ERR // 0x00000004: 校验错误  ZM_ACK_TYPE_FILE_ID_ERR, // 0x00000005: ⽂件ID错误
} ZM_POTOCOL_ACK_TYPE_EN;

enum LidarState
{
	OFFLINE = 0,
	ONLINE,
    BUSY,
	QUIT
};
enum LidarAction
{
	NONE,
	FINISH,
	START,
	STOP,
	CMD_TALK,
	CACHE_CLEAR,
};
enum PrintDevel
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR,
	MSG_ALARM
};

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


struct TX_CmdHeader
{
    char head[2];//"LH"
    uint16_t size;//cmd data crc32的长度
    uint8_t cmd;//命令类型
    uint8_t data[0];//命令数据
};
struct RX_CmdHeader
{
    char head[2];//"LH"
    uint16_t size;//cmd data crc32的长度
    uint8_t cmd;//命令类型
    uint32_t atk;//应答处理结果
    uint8_t data[0];//应答数据
};
}





#pragma pack(pop)

typedef void (*LidarCloudPointCallback)(uint16_t handle, const uint8_t msg_type, const void *data, uint16_t data_len);
typedef void (*LidarLogDataCallback)(uint16_t handle, const uint8_t msg_type, const char *data, uint16_t data_len);

#endif
