#ifndef __DATA_H__
#define __DATA_H__

#define UNUSED(x) (void)x
#define BUF_SIZE   4*1024
#define CMD_REPEAT 50

#define X_STX 0x01
#define X_EOF 0x04
#define X_ACK 0x06
#define X_NAK 0x15
#define UART_WAITTIME 100
#define CRC_POLY 0x1021
#define CRC16POLY 0x1021
enum {
    PROTOCOL_XMODEM,
    PROTOCOL_YMODEM,
};
#define CMP_MIN(a, b)       ((a) < (b) ? (a) : (b))
#define CMP_MAX(a, b)       ((a) > (b) ? (a) : (b))
#include <stdint.h>
#include<stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>

#define MAX_TIMEOUT 500                     //cmd response outtime   500ms
#define TRY_TIME 10                         //cmd try time max 10    
#define ZM_LOAD_FILE_OK                     "load file ok"
#define ZM_LOAD_FILE_NG                     "load file ng"
#define ZM_OPEN_COM_NG                      "open com ng"
#define ZM_OPEN_COM_OK                      "open com ok"
#define ZM_OPEN_COM_NG                      "open com ng"
#define ZM_CMD_TYPE_SET_BPS_OK              "change bps ok"
#define ZM_CMD_TYPE_SET_BPS_NG              "change bps ng"
#define ZM_CMD_TYPE_SET_BPS_NG_TRY          "change bps ng,is trying"

#define ZM_CMD_TYPE_SET_QUERY_VERSION_OK    "query version ok"
#define ZM_CMD_TYPE_SET_QUERY_VERSION_NG    "query version ng"
#define ZM_CMD_TYPE_START_UPDATE_OK         "start update ok"
#define ZM_CMD_TYPE_START_UPDATE_NG         "start update ng"
#define ZM_CMD_TYPE_FIRMWARE_INFO_OK        "firmware info sync ok"
#define ZM_CMD_TYPE_FIRMWARE_INFO_NG        "firmware info sync ng"
#define ZM_CMD_TYPE_FIRMWARE_DATA_OK        "firmware data sync ok"   
#define ZM_CMD_TYPE_FIRMWARE_DATA_NG        "firmware data sync ng"
#define ZM_CMD_TYPE_END_UPDATE_OK           "end update ok"
#define ZM_CMD_TYPE_END_UPDATE_NG           "end update ng"


#endif
