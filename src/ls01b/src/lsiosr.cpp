/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: serial
@filename: lsiosr.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
*******************************************************/
#include "../include/ls01b/lsiosr.h"

namespace ls {

LSIOSR * LSIOSR::instance(std::string name, int speed, int fd)
{
  static LSIOSR obj(name, speed, fd);
  return &obj;
}

LSIOSR::LSIOSR(std::string port, int baud_rate, int fd):port_(port), baud_rate_(baud_rate), fd_(fd)
{
  printf("port = %s, baud_rate = %d\n", port.c_str(), baud_rate);
}

LSIOSR::~LSIOSR()
{
  close();
}
/* 串口配置的函数 */
int LSIOSR::setOpt(int nBits, uint8_t nEvent, int nStop)
{
  struct termios newtio, oldtio;
  /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
  if (tcgetattr(fd_, &oldtio) != 0)
  {
    perror("SetupSerial 1");
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  /*步骤一，设置字符大小*/
  newtio.c_cflag |= CLOCAL;   //如果设置，modem 的控制线将会被忽略。如果没有设置，则 open()函数会阻塞直到载波检测线宣告 modem 处于摘机状态为止。
  newtio.c_cflag |= CREAD;    //使端口能读取输入的数据
  /*设置每个数据的位数*/
  switch (nBits)
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  /*设置奇偶校验位*/
  switch (nEvent)
  {
  case 'O': //奇数
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;   //使能校验，如果不设PARODD则是偶校验
    newtio.c_cflag |= PARODD;   //奇校验
    break;
  case 'E': //偶数
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'N':  //无奇偶校验位
    newtio.c_cflag &= ~PARENB;
    break;
  }
  /*设置波特率*/
  switch (baud_rate_)
  {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 230400:
    cfsetispeed(&newtio, B230400);
    cfsetospeed(&newtio, B230400);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }

  /*
   * 设置停止位
   * 设置停止位的位数， 如果设置，则会在每帧后产生两个停止位， 如果没有设置，则产生一个
   * 停止位。一般都是使用一位停止位。需要两位停止位的设备已过时了。
   * */
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  /*设置等待时间和最小接收字符*/
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  /*处理未接收字符*/
  tcflush(fd_, TCIFLUSH);
  /*激活新配置*/
  if ((tcsetattr(fd_, TCSANOW, &newtio)) != 0)
  {
    perror("serial set error");
    return -1;
  }

  return 0;
}

/* 从串口中读取数据 */
int LSIOSR::read(char *buffer, int length, int timeout)
{
  memset(buffer, 0, length);

  int	totalBytesRead = 0;
  int rc;
  char* pb = buffer;
  if (timeout > 0)
  {
    rc = waitReadable(timeout);
    if (rc <= 0)
    {
      return (rc == 0) ? 0 : -1;
    }

    int	retry = 3;
    while (length > 0)
    {
      rc = ::read(fd_, pb, (size_t)length);
      if (rc > 0)
      {
        length -= rc;
        pb += rc;
        totalBytesRead += rc;

        if (length == 0)
        {
          break;
        }
      }
      else if (rc < 0)
      {
        printf("error \n");
        retry--;
        if (retry <= 0)
        {
          break;
        }
      }

      rc = waitReadable(20);
      if (rc <= 0)
      {
        break;
      }
    }
  }
  else
  {
    rc = ::read(fd_, pb, (size_t)length);
    if (rc > 0)
    {
      totalBytesRead += rc;
    }
    else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
    {
      printf("read error\n");
      return -1;
    }
  }

  if(0)
  {
    printf("Serial Rx: ");
    for(int i = 0; i < totalBytesRead; i++)
    {
      printf("%02X ", (buffer[i]) & 0xFF);
    }
    printf("\n\n");
  }

  return totalBytesRead;
}

int LSIOSR::waitReadable(int millis)
{
  if (fd_ < 0)
  {
    return -1;
  }
  int serial = fd_;
  
  fd_set fdset;
  struct timeval tv;
  int rc = 0;
  
  while (millis > 0)
  {
    if (millis < 5000)
    {
      tv.tv_usec = millis % 1000 * 1000;
      tv.tv_sec  = millis / 1000;

      millis = 0;
    }
    else
    {
      tv.tv_usec = 0;
      tv.tv_sec  = 5;

      millis -= 5000;
    }

    FD_ZERO(&fdset);
    FD_SET(serial, &fdset);
    
    rc = select(serial + 1, &fdset, NULL, NULL, &tv);
    if (rc > 0)
    {
      rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
      break;
    }
    else if (rc < 0)
    {
      rc = -1;
      break;
    }
  }

  return rc;
}


int LSIOSR::waitWritable(int millis)
{
  if (fd_ < 0)
  {
    return -1;
  }
  int serial = fd_;

  fd_set fdset;
  struct timeval tv;
  int rc = 0;

  while (millis > 0)
  {
    if (millis < 5000)
    {
      tv.tv_usec = millis % 1000 * 1000;
      tv.tv_sec  = millis / 1000;

      millis = 0;
    }
    else
    {
      tv.tv_usec = 0;
      tv.tv_sec  = 5;

      millis -= 5000;
    }

    FD_ZERO(&fdset);
    FD_SET(serial, &fdset);

    rc = select(serial + 1, NULL, &fdset, NULL, &tv);
    if (rc > 0)
    {
      rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
      break;
    }
    else if (rc < 0)
    {
      rc = -1;
      break;
    }
  }

  return rc;
}

/* 向串口中发送数据 */
int LSIOSR::send(const char* buffer, int length, int timeout)
{
  if (fd_ < 0)
  {
    return -1;
  }

  if ((buffer == 0) || (length <= 0))
  {
    return -1;
  }

  int	totalBytesWrite = 0;
  int rc;
  char* pb = (char*)buffer;


  if (timeout > 0)
  {
    rc = waitWritable(timeout);
    if (rc <= 0)
    {
      return (rc == 0) ? 0 : -1;
    }

    int	retry = 3;
    while (length > 0)
    {
      rc = write(fd_, pb, (size_t)length);
      if (rc > 0)
      {
        length -= rc;
        pb += rc;
        totalBytesWrite += rc;

        if (length == 0)
        {
          break;
        }
      }
      else
      {
        retry--;
        if (retry <= 0)
        {
          break;
        }
      }

      rc = waitWritable(50);
      if (rc <= 0)
      {
        break;
      }
    }
  }
  else
  {
    rc = write(fd_, pb, (size_t)length);
    if (rc > 0)
    {
      totalBytesWrite += rc;
    }
    else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
    {
      return -1;
    }
  }

  if(0)
  {
    printf("Serial Tx: ");
    for(int i = 0; i < totalBytesWrite; i++)
    {
      printf("%02X ", (buffer[i]) & 0xFF);
    }
    printf("\n\n");
  }

  return totalBytesWrite;
}

int LSIOSR::init()
{
  int error_code = 0;

  fd_ = open(port_.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
  if (0 < fd_)
  {
    error_code = 0;
    setOpt(DATA_BIT_8, PARITY_NONE, STOP_BIT_1);//设置串口参数
    printf("open_port %s , fd %d OK !\n", port_.c_str(), fd_);
  }
  else
  {
    error_code = -1;
    printf("open_port %s ERROR !\n", port_.c_str());
  }
  printf("LSIOSR::Init\n");

  return error_code;
}

int LSIOSR::close()
{
  ::close(fd_);
}

std::string LSIOSR::getPort()
{
  return port_;
}

int LSIOSR::setPort(std::string name)
{
  port_ = name;
  return 0;
}

}
