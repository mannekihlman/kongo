/*
 * Axiomtek RSB10X API Library
 * User-Mode Driver	
 * Wrote by jrtiger.lee@axiomtek.com.tw
 * 		louis.huang@axiomtek.com.tw
 */

/* Library ==> /usr/lib/librsb10x.so.1.0.X */

/********************************************************/
#define SET_COM_TYPE		0x542A			
#define SET_RS485_TERM		0x542C
/* int fd, type=3, enable=1;				*/
/* 	//type	1:232;					*/
/* 	//type	2:485;					*/
/* 	//type	3:422;					*/
/* fd = open("/dev/ttymxc1", O_RDWR | O_NOCTTY);	*/
/* ioctl(fd, SET_COM_TYPE, &type);			*/
/* 	//enable	0,1: Disable, Enable		*/
/* ioctl(fd, SET_RS485_TERM, &enable);			*/
/********************************************************/
int Get_DI0(int *data);
/* Digital-Input Data store in *data			*/
int Get_DI1(int *data);
/* Digital-Input Data store in *data			*/
int Set_DO0(int data);
/* Write Data to Digital-Output Port			*/
/********************************************************/
int Get_DI0_not(int *data);
/* Digital-Input Data reverse store in *data		*/
int Get_DI1_not(int *data);
/* Digital-Input Data reverse store in *data		*/
int Set_DO0_not(int data);
/* Write reverse Data to Digital-Output Port		*/
/********************************************************/
int Set_RELAY(int data);
/* Set RELAY						*/
/* hl  0: LOW						*/
/* hl  1: HIGH						*/
/* Return 0: success    -1: fail			*/
/********************************************************/
int Contrl_WDT(int timeout,int sleep_t,int test);
/* Set WDT                                              */
/* timeout: value in seconds to cause wdt timeout/reset */
/* sleep_time: value in seconds to service the wdt      */
/* test: 0 - Service wdt with ioctl(), 1 - with write() */
/********************************************************/
int Control_LED(int num,int enable);
/* Set LED						*/
/* num : LED number,default as 1 ~4			*/
/* enable : 1 : enable , 0 : disable			*/


//removed
/********************************************************/
//int Set_COM_mode(int number, int mode, int term);
/* Set COM Port mode					*/
/* number   1: COM1					*/
/* mode 232,422,485: 1:RS232  2: RS422  3: RS485	*/
/* term   0,1: Disable, Enable				*/
/* Return 0: success    -1: fail			*/
