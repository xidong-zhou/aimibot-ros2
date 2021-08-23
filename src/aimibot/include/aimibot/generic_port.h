/**
 * @file generic_port.h
 * @brief 
 * @author Hang Zhong (zhonghanghnueeit@gmail.com)
 * @version 1.0
 * @date 2020-05-08
 * 
 * @copyright Copyright (c) 2020  aimijia tech
 * 
 * @par Modify log:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-05-08 <td>1.0     <td>Hang Zhong     <td>***
 * </table>
 */

#ifndef GENERIC_PORT_H_
#define GENERIC_PORT_H_

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
//   Generic Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Generic Port Class
 *
 * This is an abstract port definition to handle both serial and UDP ports.
 */
class Generic_Port
{
public:
	Generic_Port(){};
	virtual ~Generic_Port(){};
	virtual int Open()=0;
	virtual int Setup()=0;
	virtual int Write(const unsigned char *buffer, int length)=0;
	virtual int Read(unsigned char *buffer, int length)=0;
	virtual int Close()=0;
};


#endif // GENERIC_PORT_H_