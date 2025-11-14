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

#endif
