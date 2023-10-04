#include <sys/time.h>

//-----------------------------------------------------------------------
// The routing on the Taskit ADC16 25 pin DSUB:
//
// D7 (pin 5)  Output : Power to temperature sensor
// D6 (pin 17) Output : Step pulse to motor-controller (motor 1)
// D5 (pin 4)  Input  : Reference position switch
// D4 (pin 16) Output : Enable signal to motor-controller
// D3 (pin 3)  Input  : Reference for motor 2 (currently not used but supported in SW)
// D2 (pin 15) Output : Step pulse to motor 2 (currently not used but supported in SW)
// D1 (pin 2)  Output : Dir for motor 1 (currently not used but supported in SW)
// D0 (pin 14) Output : Dir for motor 0 (currently not used but supported in SW)

// A0 (pin 7) Temperature analog value
// A3 (pin 21) Power supply voltage divided by 10

namespace
{
  const char setPinDir[] = ":06000000d7..\r";           // Will configure all IO except D3 and D5 to outputs
  const char setOutCfg[] = ":06000100ff..\r";           // Will configure all IO to push-pull
  const char readInputRegister[] = ":0300030001..\r";   // Command for reading all IO inputs
  const char readAllADCRegisters[] = ":0400000008..\r"; // Command for reading all 8 ADC channels

  const u8 TEMP_SENSOR_PIN = 7U;
  
  u8 ourOutputs = (1 << TEMP_SENSOR_PIN);		// Temperature sensor default on

  //-----------------------------------------------------------------------
  char* SendADCCommand(const char* const command, int maxLength = sizeof(txt))
  {
    int j=0;
    txt[0] = 0;

    write(port_adc, command, strlen(command));

    struct timeval tv1;
    struct timeval tv2;
    gettimeofday(&tv1,0);

    CheckSerial(port_adc,1000);
    if(debugflag>2)
    {
        gettimeofday(&tv2,0);
        long t=(tv2.tv_sec-tv1.tv_sec)*1000000;
        t+=(tv2.tv_usec-tv1.tv_usec);
        syslog(LOG,"Time %d\n",t);
    }
    while(CheckSerial(port_adc,1) && (j<maxLength))
      {
	    j+=ReadSerial(port_adc, &txt[j],maxLength-j);
      }
    txt[j] = 0;
    if(debugflag>2)
      {
	    syslog(LOG,"Sent %s ADC returned %s",command,txt);
      }
    return txt;
  }

}

namespace Taskit
{
  //-----------------------------------------------------------------------
  void InitADC()
  {
    (void) SendADCCommand(setPinDir);
    (void) SendADCCommand(setOutCfg);
  }


  //-----------------------------------------------------------------------
  int ControlOutput(u8 activate_mask, u8 deactivate_mask)
  {
    ourOutputs &= ~deactivate_mask;
    ourOutputs |= activate_mask;
    char setOutPins[100];
    sprintf(setOutPins,":06000200%02x..\r",ourOutputs);
    const char* const p = SendADCCommand(setOutPins);
    int success = 1;
    if (strncasecmp(p,setOutPins,9))
      {
	syslog(LOG,"ControlOutput failed %s != '%s'",setOutPins,p);
	success = 0;
      }
    return success;
  }

  //-----------------------------------------------------------------------
  int GetAllADCchannels(unsigned short* result)
  {
    char* p = SendADCCommand(readAllADCRegisters);

    // Will typically return text similar to :03100000000000FF00AF010C00000000000032

    if(strlen(p) != 41)
      {
	syslog(LOG,"ADC returned invalid length %d\n",strlen(p));
	return 0;
      }
    unsigned long d;
    for(int i=0 ;i<8;i++)
      {
        const char tmp = p[5+4+i*4];
        p[5+4+i*4] = 0;
        sscanf(&p[5+i*4],"%x",&d);
	result[i] = d;
        p[5+4+i*4] = tmp;
      }
    if (debugflag>0)
      {
        syslog(LOG, "ADC: %d %d %d %d %d %d %d %d\n",
	       result[0],result[1],result[2],result[3],
	       result[4],result[5],result[6],result[7]);
      }
    return 1;
  }

  //-----------------------------------------------------------------------
  u16 GetInputs()
  {
    const char* const p = SendADCCommand(readInputRegister);
    // Will typically return text ':030200AF4C'

    unsigned long d;
    sscanf(&p[5],"%x",&d);
    d = d >> 8;                    // Remove last byte (CRC)
    return d;
  }

} // end of namespace
