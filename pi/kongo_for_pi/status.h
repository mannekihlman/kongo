#define STARTPROGRAM 1
#define STOPPROGRAM 2
#define WAIT4S 3
#define HOMEMOTOR 4
#define MEMERR 5
#define USERINT 6
#define EXECUTE 7
#define DELETE 8
#define OPENCOMERR 9
#define GPSTIMEOUT 10
#define SETCPUTIME 11
#define USEDCPUTIME 12
#define POWERON 13
#define POWEROFF 14
#define MOVEMOTOROK 15
#define MOTORDONE 16
#define ALREADYHOME 17
#define MOVEMOTOR 18
#define DOSCANDONE 19
#define DOSCANTIMEOUT 20
#define SPECREFUSE 21
#define DONEINITSPEC 22
#define INITSPEC 23
#define WRITEOK 24
#define NOTOK 25
#define CONTACTSPEC 26
#define NOCONTACTSPEC 27
#define SERIALOPEN 28
#define SERIALOPENERR 29
#define COMMAND 30
#define PAUSE 31
#define RESUME 32

void StatusPrint(unsigned char s) {
    char *ptr;
    if (debugflag == 0) return;
    switch (s) {
        case STARTPROGRAM:
            syslog(LOG, "Started Program");
            break;
        case STOPPROGRAM:
            syslog(LOG, "Exiting program");
            break;
        case WAIT4S:
            syslog(LOG, "Waiting 4 seconds");
            break;
        case HOMEMOTOR:
            syslog(LOG, "Homing motor");
            break;
        case MEMERR:
            syslog(LOG, "Not enough memory");
            break;
        case USERINT:
            syslog(LOG, "Interrupted by user");
            break;
        case EXECUTE:
            syslog(LOG, "Executing");
            break;
        case DELETE:
            syslog(LOG, "Deleting");
            break;
        case OPENCOMERR:
            syslog(LOG, "Could not open COM port\n");
            break;
        case GPSTIMEOUT:
            syslog(LOG, "GPS timeout\n");
            break;
        case SETCPUTIME:
            syslog(LOG, "Setting CPU time&date from GPS time&date\n");
            break;
        case USEDCPUTIME:
            syslog(LOG, "Used time from CPU");
            break;
        case POWERON:
            syslog(LOG, "Power ON");
            break;
        case POWEROFF:
            syslog(LOG, "Power OFF");
            break;
        case MOVEMOTOROK:
            syslog(LOG, "MoveMotor Successful");
            break;
        case MOTORDONE:
            syslog(LOG, "Motor done");
            break;
        case ALREADYHOME:
            syslog(LOG, "Motor already at home. Will make one round to reset.\n");
            break;
        case MOVEMOTOR:
            syslog(LOG, "MoveMotor");
            break;
        case DOSCANDONE:
            syslog(LOG, "doscan DONE");
            break;
        case DOSCANTIMEOUT:
            syslog(LOG, "Timeout in doscan");
            break;
        case SPECREFUSE:
            syslog(LOG, "Spectrometer refuses to scan. Reinitializing");
        case DONEINITSPEC:
            syslog(LOG, "InitSpectrometer done");
            break;
        case INITSPEC:
            syslog(LOG, "InitSpectrometer");
            break;
        case WRITEOK:
            syslog(LOG, "Writing OK");
            break;
        case NOTOK:
            syslog(LOG, "Not successful");
            break;
        case CONTACTSPEC:
            syslog(LOG, "Got contact with spectrometer");
            break;
        case NOCONTACTSPEC:
            syslog(LOG, "No contact with spectrometer");
            break;
        case SERIALOPEN:
            syslog(LOG, "Opened serial port");
            break;
        case SERIALOPENERR:
            syslog(LOG, "Could not open serial port");
            break;
        case COMMAND:
            syslog(LOG, "Command");
            break;
        case PAUSE:
            syslog(LOG, "Pause");
            break;
        case RESUME:
            syslog(LOG, "Resume");
            break;
    }
}
