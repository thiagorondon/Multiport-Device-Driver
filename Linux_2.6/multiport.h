/*
 * multiport.h
 *
 * Copyright (C) 2008 systembase
 *
 * UART registers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef MULTIPORT_H
#define MULTIPORT_H

#define TTY_MP_MAJOR        54
#define PCI_VENDOR_ID_MULTIPORT    0x14A1
#define PCI_DEVICE_ID_MP1       0x4d01
#define PCI_DEVICE_ID_MP2       0x4d02
#define PCI_DEVICE_ID_MP4       0x0004
#define PCI_DEVICE_ID_MP8       0x0008
#define PCI_DEVICE_ID_MP32      0x0032
#define MAX_MP_DEV  8
#define BD_MAX_PORT 32 	/* Max serial port in one board */
#define MP_MAX_PORT 256 /* Max serial port in one PC */

#define PORT_16C105X	2
#define PORT_16C55X		1

#define ENABLE		1
#define DISABLE		0

/* ioctls */
#define TIOCGNUMOFPORT	0x545F
#define TIOCSMULTIECHO	0x5440
#define TIOCSPTPNOECHO	0x5441

/* serial interface */
#define RS232		1 
#define RS422PTP	2
#define RS422MD		3
#define RS485NE		4
#define RS485ECHO	5

#define serial_inp(up, offset)      serial_in(up, offset)
#define serial_outp(up, offset, value)  serial_out(up, offset, value)

#endif
