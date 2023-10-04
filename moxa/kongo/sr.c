#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <syslog.h>


int debugflag=1;
 
#include "moxadevice.h"
// #include "matrix500.h"


int PORT_COM=1;
typedef unsigned char u8;
typedef unsigned short u16;


int LOG=(LOG_USER|LOG_DEBUG);


#define TASKPRIO_HIGH -20
#define TASKPRIO_LOW 10


void delay (long ns)
{
  //nanosleep(ns);
#if 1
  struct timeval tv1, tv2;
  int d=  (ns + 999)/1000;
  gettimeofday (&tv2, 0);
  tv2.tv_usec += d;
  if (tv2.tv_usec >= 1000000) {
    tv2.tv_usec -= 1000000;
    tv2.tv_sec++;
  }
  int wdg=1000000;
  do {
    gettimeofday (&tv1, 0);
    --wdg;
    if(!wdg)
      {
	printf("delay wdg %d %d %d %d\n",
	       tv1.tv_sec,tv2.tv_sec,tv1.tv_usec,tv2.tv_usec);
	return;
      }
  } while (tv1.tv_sec < tv2.tv_sec
	   || (tv1.tv_sec == tv2.tv_sec && tv1.tv_usec < tv2.tv_usec));
#endif
}

void SetTaskPrio(short prio)
{
  setpriority(PRIO_PROCESS,0,prio);
}

int GetSwitchStatus(int node)
{
  int state;
  if(node==0)
    {
      ioctl(PORT_COM,TIOCMGET,&state);
      return (state & TIOCM_CTS)!=0;  
    }
  if(node==1)
    {
      ioctl(PORT_COM,TIOCMGET,&state);
      return (state & TIOCM_CD)!=0;  
    }
}

int CheckSerial(int port,long t)
{
  int i;

  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(port,&rfds);
  tv.tv_sec=0;
  tv.tv_usec=t*1000;
  i=select(port+1,&rfds,NULL,NULL,&tv);
   //  syslog(LOG,"CheckSerial %d\n",i); 
  return(i);
}


int din()
{
  int state;
  ioctl(PORT_COM,TIOCMGET,&state);
  return (state & TIOCM_CD)!=0;  

}

int getSerial(int timeout,u8* result)
{
  if(CheckSerial(PORT_COM,timeout/1000))
    {
      return read(PORT_COM,result,1)==1;
    }
  return 0;
}

u8 scramble(u8 in)
{
  u8 ut= ~ in;
  return ut;
}

void sendSerial(u8 data)
{
  //  printf("sendSerial %d\n",data);
  write(PORT_COM,&data,1);
}



double battery=0.0;
u8 hw_pins=0;

#define port_adc PORT_COM
double temperature;

int getADC( u8 retries,u16* adout )
{
  u8 hw_version=0;
  u8 i;
  int timeout=0;
  u16 data[21];

  u8 cmd=3;
  u8 ack=0;
  u8 junk;
  if(debugflag>2)
    {
      syslog(LOG,"getADC retries=%d\n",retries);
    }
  while(retries)
    {
      int error=0;
      cmd=3;
      sendSerial(cmd);
      sendSerial(scramble(cmd) );
      if(! getSerial(1000000,&ack) )
	{
	  if(debugflag>0)
	    syslog(LOG,"getADC timeout when waiting on ACK (retries left %d)\n",retries);
	  error=1;	  
	}
      else if(ack!=cmd)
	{
	  u8 try1=0xAA;
	  u8 try2=0x55;
	  getSerial(10000,&try1);
	  getSerial(10000,&try2);
	  if(debugflag>0)
	    syslog(LOG,"getADC Incorrect ACK %02x!=%02x (%02x %02x) (retries left %d)\n",cmd,ack,try1,try2,retries);
	  error=1;
	}
      else
	{
	  u8* ptr=(u8 *)&data[0];
	  u16 crc_sum=0;
	  for(i=0;i<42;i++)
	    {
	      if(! getSerial(1000000,ptr))
		{
		  timeout=1;
		  break;
		}
	      else
		{
		  if(i<20)
		    {
		      crc_sum+=*ptr;
		    }
		  ptr++;
		}
	    }

	  if(timeout )
	    {
	      syslog(LOG,"Timeout\n");
	      error=1;
	    }
	  else
	    {
	      if(data[20]!=crc_sum)
		{
		  syslog(LOG,"ADC CRC mismatch %02x %02x\n",data[20],crc_sum);
		  error=1;
		}
	      ptr=(u8 *)&data[0];
	      for(i=0;i<20;i++)
		{
		  u8 a=scramble(ptr[i]);
		  u8 b=ptr[i+20];
		  if(a!=b )
		    {
		      syslog(LOG,"ADC Transfer fail on %d %02x %02x\n",i,a,b);
		      error=1;
		    } 
		}
	    }
	}
      SetTaskPrio(TASKPRIO_LOW);

      if(error==0)
	{
	  int bat=0;
	  int temp=3;
	  int vref6=8;
	  int ver=9;
	  
	  battery=10.0*data[bat]*0.6/data[vref6];
	  double t=100.0*data[temp]*0.6/data[vref6];
	  // sanity check of temperature
	  if( (t>243.2) && (t<373.2) )
	    {
	      temperature=t-273.2;
	    }
	  hw_version=data[ver] & 0xff;
	  hw_pins=(data[ver]>>8) & 0xff;
	  if(debugflag)
	    {
	      syslog(LOG,"Battery: %.2lf V\n",battery);
	      syslog(LOG,"Temp: %.2lf deg C\n",   temperature);
	      syslog(LOG,"Version: %d %02x\n",hw_version,hw_pins);
	    }
	  // Resize values to old format
	  for(i=0;i<9;i++)
	    {
	      adout[i]=600.0*data[i]/data[vref6];
	      printf("ADC %d %d %d\n",i,data[i]/64,adout[i]);
	    }

	  return hw_version;
	}
      retries--;
      // CollectDirt();
    }
  syslog(LOG,"getADC failed\n");
  return 0;
}


void CollectDirt()
{
  u8 dirt;
  sendSerial(0);
  if(! getSerial(100000,&dirt) )
    {
      syslog(LOG,"No dirt first time\n");
      sendSerial(0);
      if(! getSerial(100000,&dirt) )
	{
	  syslog(LOG,"No dirt second time\n");
	}
      else
	{
	  syslog(LOG,"Got the dirt ack second time %d\n",dirt);
	}
    }
  else
    {
      syslog(LOG,"Got the dirt ack first time %d\n",dirt);
    }
}

int main(int argc,char *argv[])
{
  struct termios tm;
  struct termios tm_old;
  long speed;
  char b,txt[256];
  

  PORT_COM=open(argv[1],O_RDWR|O_NOCTTY|O_NDELAY);
  if(PORT_COM<1)
    {
      exit(0);
    }


  /*set serial interface: RS-232*/
#ifdef RS232_MODE
  int interface = RS232_MODE;
  if(ioctl(PORT_COM, MOXA_SET_OP_MODE, &interface) != 0) {
    close(PORT_COM);
    return;
  }
#endif

#ifdef UC500_UART_TYPE_232
  /*set serial interface: RS-232*/
  int interface = UC500_UART_TYPE_232;
  if(ioctl(PORT_COM, UC500_SET_UART_TYPE, &interface) != 0) {
    close(PORT_COM);
    return;
  }
#endif

  cfmakeraw(&tm);  
  speed=B4800;
  cfsetispeed(&tm,speed);
  speed=B4800;
  cfsetospeed(&tm,speed);
  speed=B4800;

  tm.c_cflag = (speed | CS8 | CREAD | CLOCAL | HUPCL);
  tm.c_lflag &= ~(ECHO);
  tm.c_oflag = 0;
  tm.c_iflag = 0;
  tm.c_lflag = 0;
  tcsetattr(PORT_COM,TCSANOW,&tm);      

  long lineData;


  CollectDirt();

  while(getSerial(50000,&b))
  {// erase junk
  }

  u8 cmd=1;
  sendSerial(cmd);
  sendSerial(scramble(cmd) );

  int i=0;
  u16 data[100];
  for(;;)
    {

      if( getSerial(1000000,&b) && (i<100))
	{
	  data[i]=b;
	  // printf("A %02x %d\n",data[i],i);
	  i++;
	}
      else
	{	 
	  if(i)
	    {
	      int j;
	      for(j=0;j<i;j++)
		{
		  printf("%02x %d\n",data[j],j);
		}
	    }
	  i=0;	   
	  getADC(6,data);
	}
    }      
  tcsetattr(PORT_COM,TCSANOW,&tm_old);
  close(PORT_COM);
}
