#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <set>
#include <vector>
#include<math.h>
#include <deque>
#include<iostream>
#include<string>
#include"protocol.h"

#define getbit(x,y)   ((x) >> (y)&1)
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)            //将X的第Y位清0
#define M_PI 3.14159265358979323846


//自定义结构体
struct CmdRecord
{
    char cmd[1024];//指令
    int len;//指令长度
    unsigned short sn;//随机码
    unsigned long ts;//时间戳
    unsigned  short sign;//返回协议
    unsigned int num;//重发次数
    int mode;
};

namespace BaseAPI {
    std::string stringfilter(char *str,int num);
	bool judgepcIPAddrIsValid(const char *pcIPAddr);
	bool mask_check(const char *mask);
	bool mac_check(const char *mac);
	bool checkAndMerge(int type, char*ip, char*mask, char*gateway, int port, char*result);
    unsigned int stm32crc_8(uint8_t* ptr, unsigned int len);

}

namespace SystemAPI{
int open_serial_port(const char* name, int speed);
int closefd(int __fd,bool isSocket);
int getLastError();
uint64_t GetTimeStamp(bool isTimeStamp_M);
std::string getCurrentTime();
}
namespace CommunicationAPI {
bool uart_talk(int fd, int n, const char* cmd,int nhdr, const char* hdr_str,int nfetch, char* fetch);

}
unsigned int stm32crc(unsigned int* ptr, unsigned int len);
uint64_t getCurrentNanoseconds();
uint16_t swap16(uint16_t in);
uint16_t calcrc(uint8_t* ptr, int count);

extern "C"  int change_baud(int fd, int baud);

bool is_leap_year(int year);
int days_in_month(int year, int month);
int64_t days_since_1900(int year, int month, int day);
uint64_t datetime_to_nanoseconds(int year, int month, int day,
                                 int hour, int minute, int second, int nanosecond);

void nanoseconds_to_datetime(uint64_t nanoseconds,
                             int* year, int* month, int* day,
                             int* hour, int* minute, int* second, int* nanosecond);
#endif
