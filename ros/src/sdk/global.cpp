#include "global.h"
#include <chrono>
#include <ctime>

#ifdef _WIN32
#pragma warning(disable : 4996)
#endif

std::string BaseAPI::stringfilter(char *str, int num)
{
	int index = 0;
	for (int i = 0; i < num; i++)
	{
		if ((str[i] >= 45 && str[i] <= 58) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 97 && str[i] <= 122) || str[i] == 32 || str[i] == '_'|| str[i] == 0x0d || str[i] == 0x0a)
		{
			index++;
		}
		else
		{
			std::string arr = str;
			arr = arr.substr(0, index);
			return arr;
		}
	}
	return "";
}
unsigned int BaseAPI::stm32crc_8(uint8_t* ptr, unsigned int len)
{
    uint8_t xbit, data;
    unsigned int crc32 = 0xFFFFFFFF;
    const unsigned int polynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 7;
        data = ptr[i];
        for (uint8_t bits = 0; bits < 8; bits++)
        {
            if (crc32 & 0x80000000)
            {
                crc32 <<= 1;
                crc32 ^= polynomial;
            }
            else
                crc32 <<= 1;

            if (data & xbit)
                crc32 ^= polynomial;

            xbit >>= 1;
        }
    }
    return crc32;
}

int SystemAPI::closefd(int __fd, bool isSocket)
{
#ifdef _WIN32
	if (!isSocket)
		CloseHandle((HANDLE)__fd);
	else
		closesocket(__fd);
	return 0;
#elif __linux
	UNUSED(isSocket);
	shutdown(__fd, SHUT_RDWR);
	return close(__fd);
#endif
}
int SystemAPI::getLastError()
{
#ifdef _WIN32
	return GetLastError();
#elif __linux
	return errno;
#endif
}
uint64_t SystemAPI::GetTimeStamp(bool isTimeStamp_M)
{
	timeval  tv;
#ifdef _WIN32
	SYSTEMTIME st;
	GetLocalTime(&st);

	tv.tv_sec = (long)time(NULL);
	tv.tv_usec = st.wMilliseconds;
#elif __linux
	gettimeofday(&tv, NULL);
#endif
	if (isTimeStamp_M)
		return tv.tv_sec * 1000 + tv.tv_usec / 1000;
	else
		return tv.tv_sec;
}
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}
std::string SystemAPI::getCurrentTime()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::localtime(&t);
	char result[64] = {0};
	sprintf(result, "%04d%02d%02d_%02d%02d%02d", 1900 + tm.tm_year, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	return result;
}

uint64_t getCurrentNanoseconds() 
{
    // 使用高精度时钟（通常是 steady_clock 或 high_resolution_clock）
    auto now = std::chrono::high_resolution_clock::now();
    
    // 转换为纳秒时间戳
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}


int SystemAPI::open_serial_port(const char* name, int baud_rate)
{
	int fd = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		printf("\033[1;31m----> Open %s error\033[0m\n", name);
		return -1;
	}

	int ret;
	struct termios attrs;
	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		printf("get attrs failed");
		return -2;
	}

	/* set speed */
	// int speed = B230400;
	// // if (baudrate == 115200) speed = B115200;

	// ret = cfsetispeed(&attrs, speed);  //[baudrate]);
	// ret |= cfsetospeed(&attrs, speed); //[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1)
	{							  // parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB; // disable parity
		attrs.c_iflag &= ~INPCK;
	}
	else
	{
		attrs.c_cflag |= (PARENB | PARODD); // enable parity
		attrs.c_iflag |= INPCK;
		// if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB; // 1 stop bit
	// attrs.c_cflag |= CSTOPB;	// 2 stop bits

// Disable Hardware flowcontrol
	attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

	//	attrs.c_cc[VMIN] = 0;
	//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		printf("tcsetattr err");
		return -3;
	}

	if (change_baud(fd, baud_rate))
	{
		close(fd);
		printf("fail to set baudrate %d", baud_rate);
		return -4;
	}
	return fd;
}

bool CommunicationAPI::uart_talk(int fd, int n, const char* cmd,int nhdr, const char* hdr_str,int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);

	char buf[2048];
	int nr = read(fd, buf, sizeof(buf));
	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf + nr, sizeof(buf) - nr);
		if (n > 0)
			nr += n;
	}
	for (int i = 0; i < (int)sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0)
		{
			if (nfetch > 0)
			{
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else
				{
					strcpy(fetch, "OK");
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
			return false;
		}
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

#ifdef _WIN32
int read(int __fd, void* __buf, int __nbytes)
{
	DWORD nr;
	bool ret = ReadFile((HANDLE)__fd, __buf, __nbytes, &nr, NULL);
	if (ret == false)
		return -1;
	return nr;
}
int write(int __fd, const void* __buf, int __n)
{
	DWORD nr = 0;
	WriteFile((HANDLE)__fd, __buf, __n, &nr, NULL);
	return nr;
}
#endif

uint16_t swap16(uint16_t in)
{
    return (in >> 8) | ((in & 0xff) << 8);
}

uint16_t calcrc(uint8_t* ptr, int count)
{
    uint16_t crc, cmpt;

    crc = 0;
    //* For  all char
    while (--count >= 0)
    {

        crc = crc ^ (int)*ptr++ << 8;
        //* For All bit
        for (cmpt = 0; cmpt < 8; cmpt++)
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ CRC16POLY;
            else
                crc = crc << 1;
        }//* end bit
    }//* Frame end

    return (crc & 0xFFFF);
}
// 判断是否为闰年
bool is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

// 获取指定年份和月份的天数
int days_in_month(int year, int month) {
    static const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && is_leap_year(year)) {
        return 29;
    }
    return days_per_month[month - 1];
}
// 计算从1900年1月1日到指定日期的总天数
int64_t days_since_1970(int year, int month, int day) {
    int64_t days = 0;

    // 1900年到year-1年的天数
    for (int y = 1970; y < year; ++y) {
        days += is_leap_year(y) ? 366 : 365;
    }

    // 加上当年已过的天数
    for (int m = 1; m < month; ++m) {
        days += days_in_month(year, m);
    }
    days += day - 1;  // 减去1因为1月1日是第0天

    return days;
}

// 将年月日时分秒和纳秒转换为1900年后的纳秒时间戳
uint64_t datetime_to_nanoseconds(int year, int month, int day,
                                 int hour, int minute, int second, int nanosecond) {
    // 计算总天数
    int64_t total_days = days_since_1970(year, month, day);

    // 计算总秒数
    int64_t total_seconds = total_days * 86400LL
                            + hour * 3600LL
                            + minute * 60LL
                            + second;

    // 计算总纳秒数
    uint64_t total_nanoseconds = (uint64_t)total_seconds * 1000000000ULL
                                 + (uint64_t)nanosecond;

    return total_nanoseconds;
}

// 将纳秒时间戳转换为年月日时分秒和纳秒
void nanoseconds_to_datetime(uint64_t nanoseconds,
                             int* year, int* month, int* day,
                             int* hour, int* minute, int* second, int* nanosecond) {
    // 提取纳秒部分
    *nanosecond = nanoseconds % 1000000000ULL;
    uint64_t total_seconds = nanoseconds / 1000000000ULL;

    // 计算天数
    int64_t days = total_seconds / 86400LL;
    *second = total_seconds % 86400LL;

    // 计算时分秒
    *hour = *second / 3600;
    *minute = (*second % 3600) / 60;
    *second = *second % 60;

    // 计算年月日
    *year = 1970;
    while (days >= (is_leap_year(*year) ? 366 : 365)) {
        days -= is_leap_year(*year) ? 366 : 365;
        (*year)++;
    }

    *month = 1;
    while (days >= days_in_month(*year, *month)) {
        days -= days_in_month(*year, *month);
        (*month)++;
    }

    *day = (int)days + 1;  // 加1因为days是从0开始的
}

uint64_t SystemAPI::getTimestamp(TimeUnit unit) {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();

    switch (unit) {
        case TimeUnit::SECOND:
            return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        case TimeUnit::MILLISECOND:
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        case TimeUnit::MICROSECOND:
            return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        case TimeUnit::NANOSECOND:
            return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        default:
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }
}