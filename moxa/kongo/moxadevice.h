
/* 
 * This file is to define Moxa UC7000 series embedded device private ioctrl command. 
 * It is include LCM, KEYPAD, UART operating mode (RS232, RS422, RS485-2wire, RS485-4wires).
 * LCM device node is /dev/lcm.
 * KEYPAD device node is /dev/keypad.
 * UART device node is ttyM0 - ttyM7.
 * Copyright Moxa Technologies L.T.D.
 * History:
 * Date		Author			Comment
 * 01-13-2004	Victor Yu.		Create it.
 * 06-15-2004	Victor Yu.		Add to support UC7220 PIO library define.
 * 08-12-2004	Victor Yu.		Add to support to set special baud rate. By ioctl
 * 					MOXA_SET_SPECIAL_BAUD_RATE &
 * 					MOXA_GET_SPECIAL_BAUD_RATE.
 * 					And the termios will be B4000000. The actual baud rate
 * 					you must use above ioctl to get.
 * 09-16-2004	Victor Yu.		Add to support sWatchDog library define.
 * 01-13-2005	Victor Yu.		Add to support DIN & DOUT.
 */

#ifndef _MOXADEVICE_H
#define _MOXADEVICE_H

// following about LCM
// ioctl command define
// ioctl(fd, IOCTL_LCM_GOTO_XY, &pos)
// ioctl(fd, IOCTL_LCM_CLS, NULL)
// ioctl(fd, IOCTL_LCM_CLEAN_LINE, NULL)
// ioctl(fd, IOCTL_LCM_GET_XY, &pos)
// ioctl(fd, IOCTL_LCM_BACK_LIGHT_ON, NULL)
// ioctl(fd, IOCTL_LCM_BACK_LIGHT_OFF, NULL)
// ioctl(fd, IOCTL_LCM_AUTO_SCROLL_ON, NULL);
// ioctl(fd, IOCTL_LCM_AUTO_SCROLL_OFF, NULL);
#define IOCTL_LCM_GOTO_XY               1	// jump to position x, y
#define IOCTL_LCM_CLS                   2	// clear all screen on LCM
#define IOCTL_LCM_CLEAN_LINE            3	// clear this line on position y
#define IOCTL_LCM_GET_XY                4	// get now position x, y
#define IOCTL_LCM_BACK_LIGHT_ON         5	// let back light on
#define IOCTL_LCM_BACK_LIGHT_OFF        6	// let back light off
#define IOCTL_LCM_AUTO_SCROLL_ON	13	// when the LCM full will auto scroll up one line 
#define IOCTL_LCM_AUTO_SCROLL_OFF       14	// don't auto scroll 
typedef struct lcm_xy {	// LCM position struct define
	int     x;      // 0 - 15
	int     y;      // 0 - 7
} lcm_xy_t;

// following about KEYPAD
// ioctl command define
// ioctl(fd, IOCTL_KEYPAD_HAS_PRESS, &no)
// ioctl(fd, IOCTL_KEYPAD_GET_KEY, &value)
#define IOCTL_KEYPAD_HAS_PRESS  1	// to check any key has pressed, 0 - means no, > 0 has pressed key number
#define IOCTL_KEYPAD_GET_KEY    2	// to get has pressed key value

// keypad value define
#define KEY0                    0
#define KEY1                    1
#define KEY2                    2
#define KEY3                    3
#define KEY4                    4

// following about UART operatin mode
// ioctl command define
// ioctl(fd, MOXA_SET_OP_MODE, &mode)
// ioctl(fd, MOXA_GET_OP_MODE, &mode)
#define MOXA_SET_OP_MODE      (0x400+66)	// to set operating mode
#define MOXA_GET_OP_MODE      (0x400+67)	// to get now operating mode
// following add by Victor Yu. 08-12-2004
// ioctl(fd, MOXA_SET_SPECIAL_BAUD_RATE, &speed)
// ioctl(fd, MOXA_GET_SPECIAL_BAUD_RATE, &speed)
#define MOXA_SET_SPECIAL_BAUD_RATE      (0x400+68)	// to set special baud rate
#define MOXA_GET_SPECIAL_BAUD_RATE      (0x400+69)	// to get now special baud rate

// operating mode value define
#define RS232_MODE              0
#define RS485_2WIRE_MODE        1
#define RS422_MODE              2
#define RS485_4WIRE_MODE        3

// following just support for some products which GPIO pin.
#define MAX_GPIO		10
#define GPIO_NO_ERROR		-1 // the GPIO number error
#define GPIO_MODE_ERROR		-2 // the GPIO mode error
#define GPIO_DATA_ERROR		-3 // the GPIO data error
#define GPIO_NODE_ERROR		-4 // open GPIO device node error
#define GPIO_ERROR		-5 // some error, get error number from errno
#define GPIO_INPUT		1 // the GPIO mode is input
#define GPIO_OUTPUT		0 // the GPIO mode is output
#define GPIO_HIGH		1 // the GPIO data is high
#define GPIO_LOW		0 // the GPIO data is low
#define GPIO_OK			0 // function is OK
extern int get_gpio_mode(unsigned int pio); // 1 for input, 0 for output, GPIO from 0
extern int get_gpio_data(unsigned int pio); // 1 for high, 0 for low, GPIO from 0
extern int set_gpio_mode(unsigned int pio, int mode); // 1 for intput, 0 for output, GPIO from 0
extern int set_gpio_data(unsigned int pio, int data); // 1 for high, 0 for low, GPIO from 0

// following add by Victor Yu. 09-16-2004 to suppport sWatchDog function
extern int swtd_open(void);	// to get a file handle to control sWatchDog,<0 will be error
extern int swtd_enable(int fd, unsigned long time);	// to enable sWatchDog by application 
							// to do ACK, fd is returned mdog_open
extern int swtd_disable(int fd);	// to disable sWatchDog
extern int swtd_get(int fd, int *mode, unsigned long *time);	// to get now setting
								// mode - 1 for enable
								//	  0 for disable
								// time - now ACK time
extern int swtd_ack(int fd);	// to ACk sWatchDog
extern int swtd_close(int fd);	// to close sWatchDog

#endif	// _MOXADEVICE_H
