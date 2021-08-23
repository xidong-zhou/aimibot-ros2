/**
 * @file linux_serial.h
 * @brief Linux serial interface definition
 * 
 * Functions for opening, closing, reading and writing via serial ports
 * 
 * @author Hang Zhong (zhonghanghnueeit@gmail.com)
 * @version 1.0
 * @date 2020-05-06
 * 
 * @copyright Copyright (c) 2020  aimijia tech
 * 
 * @par Modify log:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-05-06 <td>1.0     <td>Hang Zhong     <td>***
 * </table>
 */

#ifndef LINUX_SERIAL_H_
#define LINUX_SERIAL_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "generic_port.h"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------
//#define TX_DEBUG
//#define RX_DEBUG


// ----------------------------------------------------------------------------------
//   Linux serial Port Manager Class
// ----------------------------------------------------------------------------------


/**
 * @brief Serial Port Class
 * 
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.
 */
class Serial_Port: public Generic_Port
{
public:
    Serial_Port();
    Serial_Port(const char *uart_name_, int baudrate_);
    ~Serial_Port(){};

    int Open();
    int Setup();
    int Write(const unsigned char *buffer, int length);
    int Read(unsigned char *buffer, int length);
    int Close();

private:
    int handle;
    const char *uart_name;
    int baudrate;
    bool is_open;

    void initialize_defaults();

};



#endif // LINUX_SERIAL_H_