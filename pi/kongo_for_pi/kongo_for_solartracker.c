#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dirent.h>
#include <time.h>
#include <termios.h>

#include <fcntl.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <asm/etraxgpio.h>
#include <signal.h>
#include <sys/socket.h>
#include <netdb.h>

#include <syslog.h>

int LOG = (LOG_USER | LOG_DEBUG);

/*
What pin is used for what:

ttyS0 is terminal and tx downloading channel.
ttyS3 is spectrometer serial communication.

*/

char motor1_name[] = "/dev/ttyS1";
char motor2_name[] = "/dev/ttyS2";
char spectrometer_port[] = "/dev/ttyS3";

int fdB = 0;
int fdC = 0;
int fdD = 0;

int debugflag = 1;

#ifndef min
#define   min(a, b)       ((a) < (b) ? (a) : (b))
#endif

char cfgStratoName[] = "../cfgstrat.txt";
char cfgtxtname[] = "../cfg.txt";
char cfgoncename[] = "../cfgonce.txt";

char uploadname[] = "upload.pak";
char statusname[] = "/var/status.dat";
char cmdname[] = "command.txt";

void CleanUp(int);

int maxIntTime = 5000;
int minIntTime = 10;
int stratoAngle = 5;

unsigned long hostipaddr;
char instrumentname[16];
char spectrometerType[10];
long starttime, stoptime, gpstime, gpsdate;
int gpsok = 0;
unsigned char timeisset = 0;

unsigned long serverip = 0;
char username[16];
char password[16];
int ftpiterscans;
int pcbversion;
char txt[1024];

int uploadcnt;
float compassdir = 0;
float tiltX = 0;
float tiltY = 0;
float temperature = 0.0;
int powersave = 1;
long lastidx = 0;
int skipmotor = 0;
short startchn = 0;
short stopchn = 0;
float batterylimit = 11.0;
int batteryfail = 0;
int motorswap = 0;

int motorstepscmp[2] = {0, 0};
long motorposition[2] = {0, 0};
long lastpos[2] = {0, 0};
short saved_viewangle[2];
short saved_pos[2];

int homingcnt = 0;
int measurecnt = 0;
int spectrumcnt = 0;
int measpt = 0;
int realtime = 0;
short lastexposuretime;
short lastsumcnt;
short lastchn;

short laststartchn;
short laststopchn;

int m_delay = 2;
int stepsperround[2] = {200, 0};

double gpslat, gpslon;
double gpsalt = 0;
float m_percent = 0.7;
int pchannel = -1;
float maxv;
int pause = 0;
char coneangle = 90;

#define maxlen 4096
#define buffertbytes 20000
long *smem1 = 0;
unsigned short *sbuf;
short speclen, repeat;
short solenoid;

int sun_getgpsfromfile = 0;

#define maxmeas 80

struct measurementStr {
    long pos;
    long pos2;
    int inttime;
    int realexptime;
    int intsum;
    int extsum;
    int chn;
    int repetitions;
    char name[13];
    unsigned char flag;
};
struct measurementStr meas[maxmeas];


#define TASKPRIO_HIGH -20
#define TASKPRIO_LOW 10

void SetTaskPrio(short prio) {
    setpriority(PRIO_PROCESS, 0, prio);
}


void SetPIO(unsigned char pionr, short state) {
    long pion;
    pion = 1L << pionr;
    if (state)
        ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_SETBITS), pion);
    else
        ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_CLRBITS), pion);
}

void I2Cfast(unsigned short state) {
    long pion;
    pion = 1L << 16;
    int cnt = 180;

    if (!state) {
        while (cnt--) {
            ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_SETBITS), pion);
        }
    } else {
        while (cnt--) {
            ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_CLRBITS), pion);
        }
    }
}

void I2C(unsigned short state) {
    long pion;
    pion = 1L << 16;
    int cnt = 600;

    if (!state) {
        while (cnt--) {
            ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_SETBITS), pion);
        }
    } else {
        while (cnt--) {
            ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_CLRBITS), pion);
        }
    }
}

void I2Dset(unsigned short state) {
    SetPIO(17, !state);

}

void I2Dsend(unsigned short state) {
    I2Dset(state);

}

void I2Dinput() {
    I2Dset(1);
}

int I2D() {
    int state = 0x80;
    long gb = ioctl(fdD, _IO(ETRAXGPIO_IOCTYPE, IO_READBITS));
    if ((gb & 0x00020000)) state = 0;
    return (state);
}


void msleep(unsigned short t) {
    if (t < 1) {
        I2Cfast(1);
        I2Cfast(1);
        return;
    }
    usleep(t * 1000);
}

int port_spec = 0;
int port_motor1 = 0;
int port_motor2 = 0;

#define gpsfile port_motor2

long specbaud = 115200;


int DelOldestCnt = 0;

#include "memoryfile.c"

int CheckSerial(int port, long t) {
    int i;
    if (debugflag > 3) syslog(LOG, "CheckSerial\n");

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(port, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = t * 1000;
    i = select(port + 1, &rfds, NULL, NULL, &tv);
    //  syslog(LOG,"CheckSerial %d\n",i);
    return (i);
}


void WriteSerial(void *pt, unsigned short l) {

    write(port_spec, pt, l);
}

unsigned short ReadSerial(int port, void *pt, long len) {
    if (debugflag > 3) syslog(LOG, "ReadSerial");
    int i = read(port, pt, len);
    // syslog(LOG,"%d\n",pt[0]);
    return (i);
}


void FlushSerial(int port, long t) {
    char txt;
    while (CheckSerial(port, t)) {
        ReadSerial(port, &txt, 1);
        if (debugflag > 2) syslog(LOG, "0x%0x\n", txt);
    }
}

#include "status.h"

void StatusWriter(unsigned char s) {
    FILE *f;
    unsigned char *status;
    unsigned short last, first, *p;
    if (debugflag) StatusPrint(s);

    status = txt;
    p = (unsigned short *) status;
    first = last = 4;

    f = fopen(statusname, "rb");
    if (f > (FILE *) 0) {
        fread(p, 1024, 1, f);
        fclose(f);
        first = p[0];
        last = p[1];
    }
    last++;
    if (last < 4) last = 4;
    if (last > 1023) last = 4;
    if (last == first) first++;
    if (first < 4) first = 4;
    if (first > 1023) first = 4;
    status[last] = s;
    p[0] = first;
    p[1] = last;
    f = fopen(statusname, "wb");
    if (f > (FILE *) 0) {
        fwrite(p, 1024, 1, f);
        fclose(f);
    }
}


long BaudToBits(long baud) {
    //  syslog(LOG,"BaudToBits %d\n",baud);
    switch (baud) {
        case 1200:
            return (B1200);
        case 2400:
            return (B2400);
        case 4800:
            return (B4800);
        case 9600:
            return (B9600);
        case 19200:
            return (B19200);
        case 38400:
            return (B38400);
        case 57600:
            return (B57600);
        case 115200:
            return (B115200);
        case 230400:
            return (B230400);
    }
    return (B115200);
}

void setdirection(short node, short dir) {
    if (sun_getgpsfromfile) { // we should not interfere with the solartracker
        return;
    }

    unsigned char pioD;
    long lineData;

    if (motorswap) { if (node) node = 0; else node = 1; }

    if (node == 0) {
        ioctl(port_motor1, TIOCMGET, &lineData);
        if (dir) lineData |= TIOCM_DTR;
        else lineData &= ~TIOCM_DTR;
        ioctl(port_motor1, TIOCMSET, &lineData);
    }

    if (node == 1) {
        ioctl(port_motor2, TIOCMGET, &lineData);
        if (dir) lineData |= TIOCM_DTR;
        else lineData &= ~TIOCM_DTR;
        ioctl(port_motor2, TIOCMSET, &lineData);
    }
}

void InitSerial() {
    int speed;
    int lineData;
    struct termios tm;
    port_spec = open(spectrometer_port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (port_spec == -1) {
        if (debugflag) syslog(LOG, "Could not open port %s (open() returned %d)\n", spectrometer_port, port_spec);
        StatusWriter(SERIALOPENERR);
        exit(1);
    }
    if (debugflag > 1) syslog(LOG, "Opened %s (open() returned %d)\n", spectrometer_port, port_spec);


    cfmakeraw(&tm);
    speed = BaudToBits(specbaud);
    cfsetispeed(&tm, speed);
    speed = BaudToBits(specbaud);
    cfsetospeed(&tm, speed);
    tm.c_cflag &= ~CRTSCTS;
    tm.c_cflag |= CREAD;
    tm.c_cflag |= CLOCAL;
    tcsetattr(port_spec, TCSANOW, &tm);

    if (sun_getgpsfromfile == 0) {

        port_motor1 = open(motor1_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (port_motor1 == -1) {
            if (debugflag) syslog(LOG, "Could not open port %s (open() returned %d)\n", motor1_name, port_motor1);
            StatusWriter(SERIALOPENERR);
            exit(1);
        }
        if (debugflag > 1) syslog(LOG, "Opened %s (open() returned %d)\n", motor1_name, port_motor1);


        cfmakeraw(&tm);
        speed = B4800;
        cfsetispeed(&tm, speed);
        speed = B4800;
        cfsetospeed(&tm, speed);

        tm.c_cflag &= ~CRTSCTS;
        tm.c_cflag |= CREAD;
        tm.c_cflag |= CLOCAL;
        tcsetattr(port_motor1, TCSANOW, &tm);

        port_motor2 = open(motor2_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (port_motor2 == -1) {
            if (debugflag) syslog(LOG, "Could not open port %s (open() returned %d)\n", motor2_name, port_motor2);
            StatusWriter(SERIALOPENERR);
            exit(1);
        }
        if (debugflag > 1) syslog(LOG, "Opened %s (open() returned %d)\n", motor2_name, port_motor2);
        cfmakeraw(&tm);
        speed = B4800;
        cfsetispeed(&tm, speed);
        speed = B4800;
        cfsetospeed(&tm, speed);
        tcsetattr(port_motor2, TCSANOW, &tm);
    }

    if ((fdB = open("/dev/gpiob", O_RDWR)) < 0) {
        if (debugflag) syslog(LOG, "Open error on /dev/gpiob\n");
        return;
    }


    if ((fdC = open("/dev/gpioc", O_RDWR)) < 0) {
        if (debugflag) syslog(LOG, "Open error on /dev/gpioc\n");
        return;
    }
    ioctl(fdC, _IO(ETRAXGPIO_IOCTYPE, IO_SETOUTPUT), 0x3000);


    if ((fdD = open("/dev/gpiod", O_RDWR)) < 0) {
        if (debugflag) syslog(LOG, "Open error on /dev/gpiod\n");
        return;
    }

    ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_SETOUTPUT), 0x3f000);
    ioctl(fdD, _IO(ETRAXGPIO_IOCTYPE, IO_SETINPUT), 0x3c000);

    ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_CLRBITS), 0x3d000);
    ioctl(fdB, _IO(ETRAXGPIO_IOCTYPE, IO_SETBITS), 0x2000);

    I2C(1);
    I2Dset(1);

    if (debugflag > 0)
        StatusWriter(SERIALOPEN);

    return;

}

void SetBinary() {
    if (debugflag) syslog(LOG, "Setting spectrometer to binary mode\n");
    txt[0] = 'b';
    txt[1] = 'B';
    WriteSerial(txt, 2);
    FlushSerial(port_spec, 128);
}

void ReadSerialNumber();

void ChangeBaudrate() {
    int speed;
    struct termios tm;

    int j = 0;

    FlushSerial(port_spec, 128);

    tcgetattr(port_spec, &tm);
    speed = B9600;
    cfsetispeed(&tm, speed);
    speed = B9600;
    cfsetospeed(&tm, speed);
    tcsetattr(port_spec, TCSANOW, &tm);

    CheckSerial(port_spec, 1000);
    FlushSerial(port_spec, 128);
    CheckSerial(port_spec, 4000);

    txt[0] = 'b';
    txt[1] = 'B';
    WriteSerial(txt, 2);
    j = 0;
    while (CheckSerial(port_spec, 128)) {
        ReadSerial(port_spec, txt, 1);
        j++;
    }
    if (j == 0) {
        // No answer. Can not do anything useful.
        return;
    }

    txt[0] = 'K';
    txt[1] = 0;
    txt[2] = 6;
    WriteSerial(txt, 3);
    FlushSerial(port_spec, 128);

    // close(port_spec);
    // port_spec=open(spectrometer_port,O_RDWR|O_NOCTTY|O_NDELAY);

    speed = B115200;
    cfsetispeed(&tm, speed);
    speed = B115200;
    cfsetospeed(&tm, speed);
    tcsetattr(port_spec, TCSANOW, &tm);

    txt[0] = 'K';
    txt[1] = 0;
    txt[2] = 6;
    WriteSerial(txt, 3);
    FlushSerial(port_spec, 128);

}

int ResetSpectrometer() {
    int j = 0;
    lastexposuretime = lastsumcnt = 0;
    laststopchn = laststartchn = lastchn = -1;

    while (CheckSerial(port_spec, 128)) {
        ReadSerial(port_spec, txt, 1);
        putchar(txt[0]);
        j = 1;
    }
    if (j) if (debugflag) syslog(LOG, "");
    txt[0] = 'Q';
    WriteSerial(&txt, 1);
    if (CheckSerial(port_spec, 128)) ReadSerial(port_spec, txt, 1);
    if ((txt[0] == 0x06) || (txt[0] == 21)) {
        SetBinary();
        return (0);
    }

    txt[0] = 'Q';
    WriteSerial(&txt, 1);
    if (CheckSerial(port_spec, 128)) ReadSerial(port_spec, txt, 1);
    if ((txt[0] == 0x06) || (txt[0] == 21)) {
        SetBinary();
        return (0);
    }

    // spectrometer does not respond but send binary anyway just in case

    SetBinary();
    return (1);
}

int InitCommunication() {
    if (ResetSpectrometer() == 0)
        StatusWriter(CONTACTSPEC);
    else
        StatusWriter(NOCONTACTSPEC);
    return (0);
}

unsigned short swp(unsigned short in) {
    // This code converts a short between little-endian and big-endian and vice versa
    unsigned char *p1, *p2;
    unsigned short ut;
    p1 = (unsigned char *) &ut;
    p2 = (unsigned char *) &in;
    ut = p2[1];
    ut |= p2[0] << 8;
    return (ut);
}

unsigned long longswap(unsigned long in) {
    // This code converts a longword between little-endian and big-endian and vice versa
    unsigned char *p1, *p2;
    unsigned long ut;
    p1 = (unsigned char *) &ut;
    p2 = (unsigned char *) &in;

    ut = p2[3];
    ut |= p2[2] << 8;
    ut |= p2[1] << 16;
    ut |= p2[0] << 24;
    return (ut);
}

void StripText(char *outtxt) {
    int i;
    for (i = 0; i < strlen(outtxt); i++) {
        if ((outtxt[i] < '0' || outtxt[i] > '9') && outtxt[i] != '.') outtxt[i] = ' ';
    }
}

#define BYTE char
#define WORD short
#define DWORD long

#include "mk_compress.c"

void WriteMKZY(int speclen, long *spec, short channel) {
    long last, tmp, *pt;
    int i;
    unsigned short outsiz;
    unsigned long checksum;
    unsigned short *p;

    pt = spec;
    checksum = last = *pt;
    pt++;
    for (i = 1; i < speclen; i++) {
        tmp = *pt;
        checksum += tmp;
        *pt = tmp - last;
        last = tmp;
        pt++;
    }
    p = (unsigned short *) &checksum;
    MKZY.checksum = p[0] + p[1];

    memset(sbuf, 0, buffertbytes);
    if (debugflag > 3) syslog(LOG, "mk_compress called");
    outsiz = mk_compress(spec, (unsigned char *) sbuf, speclen);
    if (debugflag > 3) syslog(LOG, "mk_compress done");
    MKZY.ident[0] = 'M';
    MKZY.ident[1] = 'K';
    MKZY.ident[2] = 'Z';
    MKZY.ident[3] = 'Y';
    strncpy(MKZY.name, meas[measpt].name, 12);
    strncpy(MKZY.instrumentname, instrumentname, 16);
    MKZY.hdrsize = sizeof(struct MKZYhdr);
    MKZY.hdrversion = hdr_version;
    MKZY.size = outsiz;
    MKZY.startc = startchn;
    MKZY.pixels = speclen;
    MKZY.starttime = starttime;
    MKZY.stoptime = stoptime;
    MKZY.date = gpsdate;
    MKZY.lat = gpslat;
    MKZY.lon = gpslon;
    MKZY.scans = meas[measpt].intsum * meas[measpt].extsum;
    MKZY.exptime = meas[measpt].realexptime;
    MKZY.viewangle = saved_viewangle[0];
    MKZY.viewangle2 = saved_viewangle[1];

    if (channel >= 256) channel -= 128;
    MKZY.channel = channel;
    MKZY.flag = solenoid;
    MKZY.altitude = gpsalt;
    MKZY.measureidx = spectrumcnt;
    MKZY.measurecnt = measurecnt - homingcnt;
    MKZY.coneangle = coneangle;

    MKZY.temperature = temperature;
    MKZY.compassdir = compassdir;
    MKZY.tiltX = tiltX;
    MKZY.tiltY = tiltY;

    if (mwrite((char *) &MKZY, sizeof(struct MKZYhdr), (char *) sbuf, outsiz) == 0)
        if (debugflag)
            syslog(LOG, "Not enough RAM memory to store work.pak file!\nYou must reduce nr of measrements in cfg.txt");

}


void SaveData(char *p, long len) {
    FILE *f;
    f = fopen("dbg.dat", "wb");
    if (f < (FILE *) 1) return;
    fwrite(p, len, 1, f);
    fclose(f);
}


unsigned long ticks;

unsigned long GetTicks() {
    long v = 0;
    char txt[2048];
    int f;
    int len;
    f = open("/proc/fasttimer", O_RDONLY);
    if (f == -1) { return (0); }
    while ((len = read(f, txt, 2047)) > 0) {
        char *pt;
        txt[len] = 0;
        pt = strstr(txt, "Current time:");
        if (pt) {
            pt = strstr(pt, ":");
            v = atoi(pt + 1);
        }
    }
    close(f);
    return (v);
}

int dosave = 0;

void SaveScan() {
    if (debugflag > 2) syslog(LOG, "SaveScan called");
    if (dosave == 0) return;
    if (debugflag > 1)
        syslog(LOG, "Date: %06ld Stoptime: %08ld\n", gpsdate, stoptime);

    WriteMKZY(speclen, smem1, meas[measpt].chn);

    dosave = 0;
    if (debugflag > 2) syslog(LOG, "SaveScan OK");
}

void GetCPUTime() {
    struct tm *tm;
    time_t t;
    time(&t);
    tm = gmtime(&t);

    unsigned long newtick;
    if (debugflag > 8) syslog(LOG, "GetCPUTime 0");

    gpstime = tm->tm_hour;
    gpstime = gpstime * 100 + tm->tm_min;
    gpstime = gpstime * 100 + tm->tm_sec;
    newtick = GetTicks();

    gpsdate = tm->tm_mday;
    gpsdate = gpsdate * 100 + tm->tm_mon + 1;
    gpsdate = gpsdate * 100 + ((tm->tm_year + 100) % 100);

    newtick -= ticks;
    newtick /= 10;
    newtick = newtick % 100;
    gpstime = gpstime * 100 + newtick;
}

#include "ftpclient.c"

int AddScan(short isum, short chn, long sleepsum) {
    char *bptr;
    char ck[1];
    unsigned short i, j, offs, mode32bit;
    long a1;


    FlushSerial(port_spec, 0);
    bptr = (char *) &sbuf[0];
    ck[0] = 0;

    txt[0] = 'S';
    WriteSerial(txt, 1);

    // save last measurement while waiting so that we can measure faster
    SaveScan();
    if (isum == 0) {
        GetCPUTime();
        starttime = gpstime;
        memset(smem1, 0, sizeof(long) * maxlen);
    }

    if (CheckSerial(port_spec, 256)) ReadSerial(port_spec, ck, 1);

    if (ck[0] != 0x02) {
        if (debugflag) syslog(LOG, "Got 0x%02x Trying again\n", ck[0]);
        FlushSerial(port_spec, 0);
        txt[0] = 'S';
        WriteSerial(txt, 1);
        ck[0] = 0;
        if (CheckSerial(port_spec, 256)) ReadSerial(port_spec, ck, 1);
        if (ck[0] != 0x02) {
            if (debugflag) syslog(LOG, "Got 0x%02x\n", ck[0]);
            return (1);
        }
    }
    if (sleepsum < 0) sleepsum = -sleepsum;
    if (debugflag > 1) syslog(LOG, "Sleeping %ld\n", sleepsum);
    if (sleepsum > 1000) msleep(sleepsum / 2);

    while (sleepsum >= 0) {
        if (debugflag > 1) syslog(LOG, "sleepsum=%d\n", sleepsum);
        if (CheckSerial(port_spec, 32767)) break;
        sleepsum -= 32767;
    }

    i = 0;
    while (CheckSerial(port_spec, 128) && i < buffertbytes) {
        j = ReadSerial(port_spec, &bptr[i], buffertbytes - i);
        i += j;
    }
    if (i < 16) {
        StatusWriter(DOSCANTIMEOUT);
        if (debugflag) {
            syslog(LOG, "i=%d\n", i);
            for (j = 0; j < i; j++) syslog(LOG, "%d %0x%04x\n", j, sbuf[j]);
        }
        return (1);
    }

    if (sbuf[0] != 0xffff) {
        if (debugflag) {
            syslog(LOG, "i=%d\n", i);
            syslog(LOG, "First word of transmission is incorrect: 0x%04x\n", sbuf[0]);
            for (j = 0; j < i; j++)
                syslog(LOG, "%d %0x%04x\n", j, sbuf[j]);
        }
        return (1);
    }
    if (debugflag > 2) { syslog(LOG, "Got %d bytes\n", i); }

    mode32bit = 0;

    offs = 7; // Default header size is 7

    if ((strstr(spectrometerType, "HR4000")) ||
        (strstr(spectrometerType, "USB4000")) ||
        (strstr(spectrometerType, "USB2000+"))) {
        // on the new spectrometers this value indicates that 32bit data follows.
        if (sbuf[1]) {
            mode32bit = 1;
        } else {
            // the offset is really a mess... these spectrometers
            // have a different header than the rest if 16 bits
            offs = 8;
        }

        // on the old spectrometers it contains the number of the ADC channel
        // so the correct setting of SPECTROMETERTYPE= is important

        // for(j=0;j<8;j++) syslog(LOG,"%d %0x%04x\n",j,sbuf[j]);

    }

    // Does the header have pixelstart and pixelstop information
    if (sbuf[6]) {
        if (mode32bit) {
            offs = 7 + 2;
        } else {
            offs = 7 + 3;
        }
    }

    speclen = i >> 1;
    speclen--;  // decrease one representing the last word (should be 0xfffd)
    if (swp(sbuf[speclen]) != 0xfffd) {
        if (debugflag) syslog(LOG, "Last word of transmission is incorrect: 0x%4x\n", sbuf[speclen]);
        return (1);
    }
    speclen -= offs;
    maxv = 0;
    if (mode32bit == 0) {
        unsigned short *pS;
        pS = (unsigned short *) (&sbuf[offs]);

        if (speclen > maxlen) speclen = maxlen;
        for (j = 0; j < speclen; j++) {
            a1 = swp(pS[j]);
            smem1[j] += a1;
            if (maxv < a1) maxv = a1;
        }
    } else {
        long *pS;
        if (debugflag > 1) syslog(LOG, "Spectrum is 32 bit precision!");
        speclen = speclen >> 1;

        // source pointer
        pS = (long *) (&sbuf[offs + 5]);

        if (debugflag > 10)
            SaveData((char *) pS, speclen * 4);

        if (speclen > maxlen) speclen = maxlen;
        for (j = 0; j < speclen; j++) {
            a1 = longswap(pS[j]);
            smem1[j] += a1;
            if (maxv < a1) maxv = a1;
        }
    }
    if (debugflag > 1) syslog(LOG, "AddScan successful");
    return (0);
}

long AvgChannels(long *s, int mid, int wid, int maxl) {
    int i, start, stop;
    long avg;

    start = mid - wid / 2;
    if (start < 0) start = 0;
    stop = mid + wid / 2;
    if (stop > maxl) stop = maxl;

    avg = 0;
    for (i = start; i < stop; i++)
        avg += s[i];
    avg /= (stop - start);
    return (avg);
}


void SendExposureTime(unsigned short inttime) {
    unsigned char t[10];
    unsigned char *p;
    int j, usenewcode;


    if (lastexposuretime == inttime) return;
    p = (char *) &inttime;

    usenewcode = 1;

    if ((strstr(spectrometerType, "HR4000")) ||
        (strstr(spectrometerType, "USB4000"))) {
        long linttime;
        linttime = inttime;
        linttime = linttime * 1000;
        if (debugflag > 1) syslog(LOG, "Exposure time: %ld microsec\n", linttime);

        if (usenewcode) {
            p = (char *) &linttime;
            t[0] = 'i';
            t[1] = p[3];
            t[2] = p[2];
            t[3] = p[1];
            t[4] = p[0];
            WriteSerial(t, 5);

            if (strstr(spectrometerType, "USB4000"))
                msleep(inttime);

            if (strstr(spectrometerType, "HR4000")) {
                // Ok for short integration times...Christoph
                if (linttime < 650000) msleep(1000);
                else if (linttime < 1800000) msleep(10000);
                else msleep(30000);
                /* It can take up to 30 seconds to change
                the exposure time using "i"! Christoph */
            }
        } else {
            t[0] = 'a';
            t[1] = 'A';
            WriteSerial(t, 2);
            msleep(500);

            memset(t, 0, 10);
            sprintf(t, "i%ld%c", linttime, 13);
            if (debugflag) syslog(LOG, "Send command %s to %s\n", t, spectrometerType);
            //added by yan
            WriteSerial(t, strlen(t));

            // Ok for short integration times...Christoph
            if (linttime < 650000) msleep(1000);
            else if (linttime < 1800000) msleep(10000);
            else msleep(30000);
            /* It can take up to 30 seconds to change
             the exposure time using "i"! Christoph */
            t[0] = 'b';
            t[1] = 'B';
            WriteSerial(t, 2);
            msleep(500);
        }
    } else {
        if (debugflag > 1) syslog(LOG, "Exposure time: %d ms\n", inttime);
        t[0] = 'I';
        t[1] = p[1];
        t[2] = p[0];
        WriteSerial(t, 3);
        msleep(inttime);
    }
    for (j = 0; CheckSerial(port_spec, 128); j++) {
        ReadSerial(port_spec, txt, 1);
        if (debugflag > 1) syslog(LOG, "%d %d\n", j, txt[0]);
    }
    lastexposuretime = inttime;
}

void SendSumCnt(short sumcnt) {
    unsigned char *p;
    int j;

    if (lastsumcnt == sumcnt) return;
    p = (unsigned char *) &sumcnt;
    if (debugflag > 1) syslog(LOG, "Internal sum count: %d\n", sumcnt);

    txt[0] = 'A';
    txt[1] = p[1];
    txt[2] = p[0];
    WriteSerial(txt, 3);
    for (j = 0; CheckSerial(port_spec, 128); j++) {
        ReadSerial(port_spec, txt, 1);
        if (debugflag > 1) syslog(LOG, "%d %d\n", j, txt[0]);
    }
    lastsumcnt = sumcnt;
}

void SendChn(short chn) {
    unsigned char *p;

    if (lastchn == chn) return;
    p = (unsigned char *) &chn;

    txt[0] = 'H';
    txt[1] = p[1];
    txt[2] = p[0];
    WriteSerial(txt, 3);
    CheckSerial(port_spec, 128);
    FlushSerial(port_spec, 1);
    lastchn = chn;
}

void SendPixelBox() {
    unsigned short sbuf;

    if ((laststartchn == startchn) && (laststopchn == stopchn)) return;

    if (startchn != stopchn) {
        unsigned char *p;

        if (debugflag) syslog(LOG, "Startchn=%d stopchn=%d\n", startchn, stopchn);
        p = (unsigned char *) &sbuf;
        txt[0] = 'P';
        sbuf = 3;
        txt[1] = p[1];
        txt[2] = p[0];
        sbuf = startchn;
        txt[3] = p[1];
        txt[4] = p[0];
        sbuf = stopchn;
        txt[5] = p[1];
        txt[6] = p[0];
        sbuf = 1;
        txt[7] = p[1];
        txt[8] = p[0];
        WriteSerial(txt, 9);
        FlushSerial(port_spec, 128);

        laststartchn = startchn;
        laststopchn = stopchn;
    }
}

int initspectrometer(short chn, short sumcnt) {
    long inttime;
    long m, digitalnoise, maxcounts;
    short stest, ltest;
    float a, b, c, d;

    if (debugflag > 0)
        StatusWriter(INITSPEC);

    FlushSerial(port_spec, 0);
    SendChn(chn);
    SendPixelBox();

    inttime = meas[measpt].inttime;
    meas[measpt].realexptime = inttime;
    if (inttime == 0) {
        inttime = meas[0].realexptime;
        if (inttime < 0) inttime = -inttime;
        if (inttime == 1) inttime = maxIntTime;
        meas[measpt].realexptime = -inttime;
        if (debugflag) syslog(LOG, "Using zenith exposure time: %ld ms\n", inttime);
    } else if (inttime < 0) {
        SendSumCnt(1);
        stest = 0;
        if (strstr(spectrometerType, "HR4000")) {
            stest = 10;
            ltest = 300;
            maxcounts = 16384;
            SendExposureTime(ltest);

            AddScan(0, chn, ltest);
            /* Throw the first scan away
             I added this because I was sometimes getting too much
             noise in this first spectrum. Now I throw it out...Christoph */
        }
        if (strstr(spectrometerType, "USB4000") ||
            strstr(spectrometerType, "USB2000+")) {
            stest = 10;
            ltest = 300;
            maxcounts = 65536;
            SendExposureTime(ltest);

            AddScan(0, chn, ltest);
            /* Throw the first scan away
             I added this because I was sometimes getting too much
             noise in this first spectrum. Now I throw it out...Christoph */
        }

        // do this code as the default (for S2000 and USB2000)
        if (stest == 0) {
            stest = 10;
            ltest = 50;
            maxcounts = 4096;
            SendExposureTime(ltest);
        }
        if (AddScan(0, chn, ltest)) {
            inttime = 20;
        } else {
            if (pchannel != -1) {
                maxv = AvgChannels(smem1, pchannel, 10, maxlen);
            }
            m = maxv;
            if (debugflag > 0) syslog(LOG, " Maxvalue: %d\n", m);

            SendExposureTime(stest);
            AddScan(0, chn, stest);
            if (pchannel != -1) {
                maxv = AvgChannels(smem1, pchannel, 10, maxlen);
            }
            digitalnoise = maxv;
            if (debugflag > 0) syslog(LOG, " Maxvalue: %d\n", digitalnoise);

            if (m < 1 || digitalnoise == m) {
                inttime = maxIntTime;
                if (debugflag) syslog(LOG, "Calculated exposure time: %ld ms\n", inttime);
            } else {
                a = maxcounts - digitalnoise;
                b = ltest - stest;
                c = m - digitalnoise;
                d = m_percent * a * b / c;
                inttime = d;
                inttime += stest;
                // Old code: inttime=(4096.0-digitalnoise)*40/(m-digitalnoise)*m_percent;

                if (debugflag) syslog(LOG, "Calculated exposure time: %ld ms", inttime);
                if (inttime > maxIntTime) {
                    inttime = maxIntTime;
                    if (debugflag) syslog(LOG, " but forced to %ld ms\n", inttime);
                } else if (inttime < minIntTime) {
                    if (m < 200) inttime = maxIntTime;
                    else inttime = minIntTime;
                    if (debugflag) syslog(LOG, " but forced to %ld ms\n", inttime);
                }

            }
            meas[measpt].realexptime = -inttime;

        }
    }
    SendSumCnt(sumcnt);

    // Bug workaround for USB2000+
    if ((lastexposuretime < 655) && (inttime >= 655)) {
        SendExposureTime(1);
        AddScan(0, chn, stest);
    }
    SendExposureTime(inttime);

    if (debugflag > 1) StatusWriter(DONEINITSPEC);

    return (0);
}


void TaskScheduling(int state) {
    //  sched_yield();
}

int rotator[2] = {0, 0};
unsigned char outbyte = 0xc0;

void SendMotor(int power) {
    int i, timout, val;
    unsigned char b;

    if (pcbversion < 3) return;

    b = outbyte & 0xcc;


    if (power == 0) b |= 0x33;
    if (power == 1) b |= 0x02;

    if (debugflag > 4) syslog(LOG, "SendMotor 0x%02x\n", b);

    TaskScheduling(0);
    SetTaskPrio(TASKPRIO_HIGH);
    I2Dinput();
    I2C(1);
    if (I2D()) {
        if (debugflag) syslog(LOG, "Error I2D already set\n");
    }
    I2Cfast(0);
    for (timout = 10000; (!I2D()) && timout; timout--) {}
    I2C(1);
    if (!timout) {
        TaskScheduling(1);
        if (debugflag) syslog(LOG, "SendMotor 1 timeout.");
        return;
    }

    // after we have put clock high it is ok to assert I2C
    I2Dsend(0);
    I2C(0);
    I2C(1);
    I2C(0);

    for (i = 0; i < 8; i++) {
        val = 0;
        if (b & 0x01) val = 1;
        b = b >> 1;
        I2Dset(val);

        I2C(1);
        I2Cfast(0);
    }

    I2Cfast(1);
    I2Dset(1);
    I2Dinput();
    I2C(0);
    for (timout = 10000; (!I2D()) && timout; timout--) {}
    I2C(1);
    TaskScheduling(1);
    SetTaskPrio(TASKPRIO_LOW);

    if (!timout) {
        if (debugflag) syslog(LOG, "SendMotor 2 timeout.");
        return;
    }
}


void SwitchRelay(unsigned short state) {
    long lineData;
    SetPIO(15, state);
    if (state == 0) SendMotor(0);
}

short lastsolenoid = -1;


void SendSolenoid(char stat) {
    int timout;
    if (debugflag > 1) syslog(LOG, "Meas %d flag=%d solenoid=%d\n", measpt, meas[measpt].flag, solenoid);

    if (pcbversion >= 3) {
        outbyte |= 0x30;
        if (stat & 1) outbyte &= 0xef;
        if (stat & 2) outbyte &= 0xdf;
        if (powersave == 1) SendMotor(1);
        if (powersave == 2) SendMotor(0);
        if (powersave == 0) SendMotor(3);
    }
}

int doscan() {
    short isum, tmpsolenoid;
    long t;
    tmpsolenoid = 0;
    if ((meas[measpt].flag & 0x07) == 7) tmpsolenoid = 3;
    if ((meas[measpt].flag & 0x07) == 4) {
        tmpsolenoid = meas[measpt].flag & 0x02;
        if ((repeat & 1) == 0) tmpsolenoid |= 1;
    } else {
        tmpsolenoid = meas[measpt].flag & 0x03;
    }
    if (tmpsolenoid != lastsolenoid) {
        SendSolenoid(tmpsolenoid);
        msleep(100); // always sleep to allow the solenoid to settle
        lastsolenoid = tmpsolenoid;
    }

    for (isum = 0; isum < meas[measpt].extsum && (!KbdCheck()); isum++) {
        if (debugflag > 1) syslog(LOG, "isum=%d\n", isum);
        t = meas[measpt].realexptime;
        t *= meas[measpt].intsum;
        if (AddScan(isum, meas[measpt].chn, t)) {
            StatusWriter(SPECREFUSE);
            FlushSerial(port_spec, 100);
            return (1);
        }
        if (debugflag) syslog(LOG, "Got %d values. Maxvalues: %g\n", speclen, maxv);
    }
    GetCPUTime();
    stoptime = gpstime;
    dosave = 1;
    solenoid = tmpsolenoid;
    if (debugflag > 1) StatusWriter(DOSCANDONE);

    saved_viewangle[0] = saved_pos[0] * 360.0 / abs(stepsperround[0]);
    if (stepsperround[1]) saved_viewangle[1] = saved_pos[1] * 360.0 / abs(stepsperround[1]);
    else saved_viewangle[1] = 0;

    return (0);
}

short GetADC(unsigned short *adout) {
    unsigned short *ad;
    short version, i, j, ok, flag;
    unsigned char *gpstxt;
    gpstxt = (unsigned char *) sbuf;
    gpstxt[0] = gpstxt[1] = 0;
    version = -1;

    for (i = 0; i < 8; i++) adout[i] = 0x0;
    ok = 0;
    ad = (unsigned short *) &gpstxt[4];

    SetTaskPrio(TASKPRIO_HIGH);
    for (j = 0; j < 4; j++) {
        flag = 0;
        if (!SendMode(7))
            if (GetLogData(gpstxt, 200)) {
                if (gpstxt[0] == 'V') {
                    version = gpstxt[1] - '0';
                    if (version >= 4) flag = 1;
                }
            }
        if (flag) {
            for (i = 0; i < 8; i++) {
                if (ad[i] & 0xf000) flag = 0;
            }
        }
        if (flag) {
            if (version == 4) {
                for (i = 0; i < 4; i++) {
                    adout[i + i] += ad[i] & 0x0fff;
                    adout[i + i + 1] += ad[i + 4] & 0x0fff;
                }
            } else {
                for (i = 0; i < 4; i++) {
                    adout[i + i] += ad[i + 8] >> 4;
                    adout[i + i + 1] += ad[i + 12] >> 4;
                }
            }
            ok++;
            if (version > 4) break;
        }
    }
    if (version >= 4 && ok) {
        for (i = 0; i < 8; i++) adout[i] /= ok;
    }

    if (debugflag > 0) {
        char m[8];
        strcpy(txt, "ADC:");
        for (i = 0; i < 8; i++) {
            sprintf(m, " %04d", adout[i]);
            strcat(txt, m);
        }
        syslog(LOG, txt);
    }
    SetTaskPrio(TASKPRIO_LOW);

    return (version);
}

void CheckBattery() {
    float v;

    if (pcbversion < 4) return;

    do {
        GetADC(&MKZY.ADC[0]);
        v = MKZY.ADC[0] * 0.01;
        if (v == 0.0) {
            if (debugflag) syslog(LOG, "No ADC installed");
            batteryfail = 0;
            return;
        }
        if (debugflag > 0) syslog(LOG, "Battery: %.2f V\n", v);
        if (v < batterylimit) {
            if (batteryfail == 0) {
                batteryfail = 1;
                if (debugflag) syslog(LOG, "Battery is low. Turning OFF\n", v);
                SwitchRelay(0);
            }
            msleep(5000);
        } else {
            if (batteryfail == 1) {
                batteryfail = 0;
                if (debugflag) syslog(LOG, "Battery recovered. Turning ON\n", v);
                SwitchRelay(1);
            }
        }
    } while (batteryfail);
    if ((strstr(spectrometerType, "HR4000")) ||
        (strstr(spectrometerType, "USB4000"))) {
    } else {
        if ((MKZY.ADC[2] > 2432) && (MKZY.ADC[2] < 3732)) {
            temperature = 0.1 * MKZY.ADC[2];
            temperature -= 273.2;
            if (debugflag > 0) syslog(LOG, "Temperature: %.2f deg C\n", temperature);
        }
    }
}

void dopulses(short node0, short node1) {
    if (sun_getgpsfromfile) { // we should not interfere with the solartracker
        return;
    }

    if (motorswap) {
        short node = node0;
        node0 = node1;
        node1 = node;
    }


    if (node0) {
        long lineData;
        ioctl(port_motor1, TIOCMGET, &lineData);
        lineData |= TIOCM_RTS;
        ioctl(port_motor1, TIOCMSET, &lineData);

        msleep(m_delay);
        lineData &= ~TIOCM_RTS;
        ioctl(port_motor1, TIOCMSET, &lineData);
    }

    if (node1) {
        long lineData;
        ioctl(port_motor2, TIOCMGET, &lineData);
        lineData |= TIOCM_RTS;
        ioctl(port_motor2, TIOCMSET, &lineData);

        msleep(m_delay);
        lineData &= ~TIOCM_RTS;
        ioctl(port_motor2, TIOCMSET, &lineData);
    }
    msleep(m_delay);

}


void sendsteps(long *steps) {
    short node, nodecnt;

    if (skipmotor == 2) return;

    if (pcbversion >= 3) {
        for (nodecnt = 0; nodecnt < 2; nodecnt++) {
            if (steps[nodecnt]) {
                if (steps[nodecnt] == 1) rotator[nodecnt]++;
                else rotator[nodecnt]--;
                node = nodecnt;
                if (motorswap) { if (nodecnt) node = 0; else node = 1; }

                if (node == 0) outbyte &= 0xf0;
                if (node == 1) outbyte &= 0x0f;
                switch (rotator[nodecnt] & 0x03) {
                    case 0:
                        /*
                     if(node==0) outbyte|=0x00;
                     if(node==1) outbyte|=0x00;
                   */
                        break;
                    case 1:
                        if (node == 0) outbyte |= 0x04;
                        if (node == 1) outbyte |= 0x40;
                        break;
                    case 2:
                        if (node == 0) outbyte |= 0x0c;
                        if (node == 1) outbyte |= 0xc0;
                        break;
                    case 3:
                        if (node == 0) outbyte |= 0x08;
                        if (node == 1) outbyte |= 0x80;
                        break;
                }
            }
        }
        SendMotor(3);
    }
}

void HomeMotor(short node, short dir) {
    int cnt, status;
    long steps[2];
    SetTaskPrio(TASKPRIO_LOW);

    steps[0] = steps[1] = 0;
    if (GetSwitchStatus(node)) {
        steps[node] = 1;
        // do steps forward until out of reference switch
        StatusWriter(ALREADYHOME);
        while (GetSwitchStatus(node)) {
            setdirection(node, steps[node]);
            dopulses(steps[0], steps[1]);
            sendsteps(steps);
        }
    }
    steps[node] = dir;
    cnt = 0;
    do {
        cnt++;
        if (m_delay) {
            if ((cnt % 1000) == 0) {
                KbdCheck();
                if (debugflag) syslog(LOG, "%d steps\n", cnt);
                CheckBattery();
            }
        } else {
            if ((cnt % 20000) == 0) {
                KbdCheck();
                if (debugflag) syslog(LOG, "%d steps\n", cnt);
                CheckBattery();
            }
        }
        status = GetSwitchStatus(node);
        if (status == 0) {
            setdirection(node, dir);
            dopulses(steps[0], steps[1]);
            sendsteps(steps);
        }
    } while ((status == 0));
    motorposition[node] = motorstepscmp[node];

    if (status) StatusWriter(MOTORDONE);
    if (powersave == 1) SendMotor(1);
    if (powersave == 2) SendMotor(0);

    return;
}

int MoveMotor(long p1, long p2) {
    int node, domove, detectref;
    short lastswitch[2];
    short newswitch[2];
    long pos[2], d[2], steps[2];

    saved_pos[0] = p1;
    saved_pos[1] = p2;

    if (skipmotor == 1) return (0);

    if (debugflag > 1) StatusWriter(MOVEMOTOR);
    pos[0] = p1;
    pos[1] = p2;
    SetTaskPrio(TASKPRIO_LOW);
    for (node = 0; node < 2; node++) {
        if (stepsperround[node]) {
            d[node] = motorstepscmp[node];
            detectref = 0;
            if (stepsperround[node] > 0) {
                detectref = 1;
                while (d[node] < 0) d[node] += stepsperround[node];
                while (motorposition[node] < 0) motorposition[node] += stepsperround[node];
                while (pos[node] < 0) pos[node] += stepsperround[node];
                motorposition[node] = motorposition[node] % stepsperround[node];
                pos[node] = pos[node] % stepsperround[node];
                d[node] = d[node] % stepsperround[node];
            }
            if (motorposition[node] != pos[node]) {
                if (detectref) lastswitch[node] = GetSwitchStatus(node);
                else lastswitch[node] = 0;
            }
            if (debugflag)
                syslog(LOG, "MoveMotor%d newpos %d oldpos %d stepscomp %d\n", node, pos[node], motorposition[node],
                       d[node]);
        }
    }
    do {
        domove = 0;
        for (node = 0; node < 2; node++) {
            steps[node] = 0;
            if (stepsperround[node]) {
                detectref = 0;
                if (stepsperround[node] > 0) {
                    detectref = 1;
                    while (motorposition[node] < 0) motorposition[node] += stepsperround[node];
                    motorposition[node] = motorposition[node] % stepsperround[node];
                }
                if (motorposition[node] != pos[node] && detectref) {
                    // The following behaviour with lastswitch will assure that
                    // we have at least one step where the switch is not on before we detect it next time
                    newswitch[node] = GetSwitchStatus(node);
                    if (lastswitch[node] == 0) {
                        if (newswitch[node]) {
                            motorposition[node] = d[node];
                            if (debugflag) syslog(LOG, "Found ref pos on node %d\n", node);
                        }
                    }
                    lastswitch[node] = newswitch[node];
                }
                if (motorposition[node] != pos[node]) {
                    // wanted position not reached yet

                    domove = 1;
                    steps[node] = 1;
                    // Check if both directions are allowed
                    if (stepsperround[node] < 0) {
                        if (pos[node] < motorposition[node]) steps[node] = -1;
                    }
                    setdirection(node, steps[node]);
                    if (steps[node] > 0) motorposition[node]++;
                    else motorposition[node]--;
                }

            }
        }
        if (domove) {

            dopulses(steps[0], steps[1]);
            sendsteps(steps);
        }
    } while (domove);
    if (debugflag > 1) StatusWriter(MOVEMOTOROK);
    if (powersave == 1) SendMotor(1);
    if (powersave == 2) SendMotor(0);

    return (0);
}

int ReadSettingFile(char *filename) {
    char *pt;
    FILE *fil;
    char nl[2] = {0x0a, 0};
    char lf[2] = {0x0d, 0};
    short flag;
    char temp[30];

    fil = fopen(filename, "r");
    if (fil < (FILE *) 1) {
        if (measurecnt) if (debugflag) syslog(LOG, "Could not open file %s\n", filename);
        measurecnt = 0;
        return (-1);
    }
    homingcnt = measurecnt = 0;

    while (fgets(txt, sizeof(txt) - 1, fil)) {

        if (strlen(txt) > 4 && txt[0] != '%') {

            if (pt = strstr(txt, "SUN_GETGPSFROMFILE=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &sun_getgpsfromfile);
            }
            if (pt = strstr(txt, "INSTRUMENTNAME=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s", temp);
                strncpy(instrumentname, temp, 16);
            }
            if (pt = strstr(txt, "SPECTROMETERTYPE=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s", temp);
                strncpy(spectrometerType, temp, 10);
            }
            if (pt = strstr(txt, "SERVER=")) {
                struct sockaddr_in address;
                struct hostent *hname;

                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s %s %s %d",
                       temp, username, password, &ftpiterscans);
                hname = gethostbyname(temp);
                memcpy(&address.sin_addr,
                       hname->h_addr, hname->h_length);
                serverip = address.sin_addr.s_addr;
            }
            if (pt = strstr(txt, "STEPSPERROUND=")) {
                int j;
                pt = strstr(txt, "=");
                j = sscanf(&pt[1], "%d %d", &stepsperround[0], &stepsperround[1]);
                if (j < 2) stepsperround[1] = 0;
            }

            if (pt = strstr(txt, "CONEANGLE=")) {
                int j;
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &j);
                coneangle = j;
            }
            if (pt = strstr(txt, "DELAY=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &m_delay);
            }
            if (pt = strstr(txt, "STARTCHN=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &startchn);
            }
            if (pt = strstr(txt, "DEBUG=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &debugflag);
            }
            if (pt = strstr(txt, "PCBVERSION=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &pcbversion);
            }
            if (pt = strstr(txt, "POWERSAVE=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &powersave);
                if (debugflag > 1) syslog(LOG, "POWERSAVE=%d\n", powersave);
            }
            if (pt = strstr(txt, "STOPCHN=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &stopchn);
            }
            if (pt = strstr(txt, "REALTIME=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &realtime);
            }
            if (pt = strstr(txt, "FTPTIMEOUT=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &ftptimeout);
            }
            if (pt = strstr(txt, "MAXINTTIME=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &maxIntTime);
            }
            if (pt = strstr(txt, "MININTTIME=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &minIntTime);
            }
            if (pt = strstr(txt, "STRATOANGLE=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &stratoAngle);
            }

            if (pt = strstr(txt, "MOTORANGLECOMP=")) {
                float anglecmp[2] = {0, 0};
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%f %f", &anglecmp[0], &anglecmp[1]);
                motorstepscmp[0] = -anglecmp[0] * abs(stepsperround[0]) / 360.0;
                motorstepscmp[1] = -anglecmp[1] * abs(stepsperround[1]) / 360.0;

            }
            if (pt = strstr(txt, "MOTORSTEPSCOMP=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d %d", &motorstepscmp[0], &motorstepscmp[1]);
            }

            if (pt = strstr(txt, "MEAS=")) {
                pt = strstr(txt, "=");
                if (measurecnt < maxmeas - 1) {
                    int i = measurecnt;
                    if (strstr(pt, "HOMEMOTOR")) {
                        sscanf(pt + 1, "%d %d", &meas[i].pos, &meas[i].pos2);
                        meas[i].extsum = meas[i].intsum = 0;
                        homingcnt++;
                    } else {
                        meas[i].repetitions = 1;
                        flag = 7;
                        if (stepsperround[1] == 0) {
                            sscanf(pt + 1, "%d %d %d %d %d %s %d %d" \
, &meas[i].pos, &meas[i].inttime, &meas[i].intsum, \
                                 &meas[i].extsum, &meas[i].chn, temp, &meas[i].repetitions, &flag);
                        } else {
                            sscanf(pt + 1, "%d %d %d %d %d %d %s %d %d" \
, &meas[i].pos, &meas[i].pos2, &meas[i].inttime, &meas[i].intsum, \
                                 &meas[i].extsum, &meas[i].chn, temp, &meas[i].repetitions, &flag);
                        }
                        if (meas[i].intsum < 1) {
                            if (debugflag)
                                syslog(LOG, "MEAS(%d) Warning intsum=%d forced to 1\n", i, meas[i].intsum);
                            meas[i].intsum = 1;
                        }
                        if (meas[i].extsum < 1) {
                            if (debugflag)
                                syslog(LOG, "MEAS(%d) Warning extsum=%d forced to 1\n", i, meas[i].extsum);
                            meas[i].extsum = 1;
                        }
                        temp[11] = 0;
                        strcpy(meas[i].name, temp);
                        //printf("%d %d %s\n",i,measurecnt,meas[i].name);
                        meas[i].flag = flag;
                    }
                    measurecnt++;

                } else if (debugflag)
                    syslog(LOG, "Maximum measurements exceeded");
            }
            if (pt = strstr(txt, "PERCENT=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%f", &m_percent);
            }
            if (pt = strstr(txt, "CHANNEL=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &pchannel);
                if (debugflag > 1) syslog(LOG, "CHANNEL=%d\n", pchannel);
            }
            if (pt = strstr(txt, "SPECBAUD=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%ld", &specbaud);
            }
            if (pt = strstr(txt, "BATTERYLIMIT=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%f", &batterylimit);
                if (debugflag > 1) syslog(LOG, "BATTERYLIMIT=%f\n", batterylimit);
            }
            if (pt = strstr(txt, "MOTORSWAP=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &motorswap);
                if (debugflag > 1) syslog(LOG, "MOTORSWAP=%d\n", motorswap);
            }
            if (pt = strstr(txt, "COMPASS=")) {
                float c;
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%f %f %f", &c, &tiltX, &tiltY);
                // a value below 360 indicates manual compass

                if (compassdir < 3600.0) compassdir = fmod(c * 10.0, 3600.0);
                tiltX *= 10.0;
                tiltY *= 10.0;
                if (debugflag > 1)
                    syslog(LOG, "COMPASS=%.1f %.1f %.1f %.1f\n", compassdir * 0.1, tiltX * 0.1, tiltY * 0.1);
            }
            if (pt = strstr(txt, "SKIPMOTOR=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &skipmotor);
            }
        }
    }
    fclose(fil);

    return (0);
}


int GetLogData(unsigned char *gpstxt, int wait) {
    int timout, i, j, c;
    gpstxt[0] = 0;

    syslog(LOG, "GetLogData I2D=%d\n", I2D());

    I2Dinput();
    I2C(0);
    for (timout = wait / 100; !I2D() && timout; timout--) { msleep(100); }

    TaskScheduling(0);
    I2C(1);
    if (!timout) {
        TaskScheduling(1);
        if (debugflag > 0) syslog(LOG, "GetLogData timeout. Data not low");
        return (0);
    }
    else {
        for (i = 0; i < 62; i++) {
            for (j = 0, c = 0; j < 8; j++) {
                I2C(0);
                c = c >> 1;
                c |= I2D();
                I2C(1);
            }
            if (c == '*') {
                gpstxt[i] = 0;
                i = 62;
            }
            else gpstxt[i] = c;
        }
        gpstxt[i] = 0;
    }
    TaskScheduling(1);
    if (debugflag > 8) syslog(LOG, gpstxt);
    return (i);
}

int SendMode(int mode) {
    int timout;

    I2Dinput();
    I2C(0);
    for (timout = 10000; !I2D() && timout; timout--) {}
    I2Dsend(mode & 0x1);

    I2C(1);
    if (!timout) {
        if (debugflag) syslog(LOG, "SendMode timeout. Data not low");
        return (-1);
    }

    // after we have put clock high it is ok to assert I2D
    I2C(0);
    I2Dsend(mode & 0x2);

    I2C(1);
    I2C(0);
    if (mode >= 3) {
        I2Dsend(mode & 0x4);
        I2C(1);

        I2C(0);
    }
    I2C(1);

    I2Dinput();
    return (0);
}


short GetVersion() {
    short version;
    unsigned char *gpstxt;

    gpstxt = (unsigned char *) sbuf;
    gpstxt[0] = gpstxt[1] = 0;
    version = -1;

    SetTaskPrio(TASKPRIO_HIGH);
    if (!SendMode(7)) {
        if (GetLogData(gpstxt, 200)) {
            if (gpstxt[0] == 'V') {
                version = gpstxt[1] - '0';
                pcbversion = version;
            } else {
                int i;
                for (i = 0; i < 62; ++i) {
                    printf("%02x ", gpstxt[i]);
                }
                puts("");
            }

        }
    }
    SetTaskPrio(TASKPRIO_LOW);
    syslog(LOG, "PCB version is set to %d\n", pcbversion);

    return (version);
}

void GetCompass() {
    unsigned char *gpstxt;
    float cmp;
    gpstxt = (unsigned char *) sbuf;

    cmp = compassdir;
    SetTaskPrio(TASKPRIO_HIGH);
    if (SendMode(3)) {
        if (debugflag)
            syslog(LOG, "Timeout 1 in GetCompass");
    } else {
        if (!GetLogData(gpstxt, 500)) {
            if (debugflag)
                syslog(LOG, "Timeout 2 in GetCompass");
        } else {
            cmp = gpstxt[0] + gpstxt[1] * 256;
            cmp *= 25.0 / 24.0;
            if (cmp > 360) cmp -= 360;
            if (debugflag > 0) syslog(LOG, "Compass %.1f\n", cmp);
            cmp += 360.0;
            cmp *= 10.0;
            compassdir = cmp;
        }
    }
    SetTaskPrio(TASKPRIO_LOW);
}

int GetSwitchStatus(int node) {
    unsigned short state = 0;
    unsigned char pio;
    unsigned long piobit, lineData;

    if (motorswap) { if (node) node = 0; else node = 1; }


    if (node == 0) {
        lineData = ioctl(fdD, _IO(ETRAXGPIO_IOCTYPE, IO_READBITS));

        //  ioctl(gpsfile,TIOCMGET,&lineData);
        if (lineData & (1L << 14)) return (0);
        else return (1);
    }
    if (node == 1) {
        //  ioctl(port_spec,TIOCMGET,&lineData);
        lineData = ioctl(fdC, _IO(ETRAXGPIO_IOCTYPE, IO_READBITS));
        if (lineData & (1L << 15)) return (1);
        else return (0);
    }
    //  if( lineData & TIOCM_DSR ) return(1);
    //  else return(0);
}

void ResyncTimer() {
    int i;
    time_t s1, s2;

    time(&s1);
    do {
        time(&s2);
    } while (s1 == s2);
    ticks = GetTicks();

}

void GetGPSString(char *match, int wait, char *out, int buflength) {
    char ck;
    int cnt = 0;
    out[0] = 0;
    if (debugflag)
        syslog(LOG, "GetGPSString");

    while (CheckSerial(gpsfile, wait)) {
        ReadSerial(gpsfile, &ck, 1);
        if (ck == match[cnt]) {
            cnt++;
            if (match[cnt] == 0) break;
        } else cnt = 0;
    }
    if (cnt == 0) return;
    cnt = 0;
    while (CheckSerial(gpsfile, 128)) {
        ReadSerial(gpsfile, &ck, 1);
        if (ck == '*' || ck == 0x0d || ck == 0x0a) break;
        out[cnt] = ck;
        cnt++;
        if (cnt == buflength) break;
    }
    out[cnt] = 0;
}

int GetGPS3() {
    long dd, ggaok, rmcok, date;
    double a, lat, lon;
    long cnt;
    char *pt;
    int iter, retv, flag;
    char *gpstxt;

    if (debugflag > 2) syslog(LOG, "GetGPS3");
    gpstxt = (char *) sbuf;
    gpstxt[0] = 0;
    iter = 6;
    lat = lon = rmcok = ggaok = 0;

    if (!SendMode(1)) {
        if (!GetLogData(gpstxt, 1500))
            return (0);
        else {
            if ((gpstxt[0] < '0' || '9' < gpstxt[0]) && gpstxt[0] != ',') {
                if (debugflag)
                    syslog(LOG, "Corrupt RMC data %c %d\n", gpstxt[0], gpstxt[0]);
                return (0);
            }
            dd = 0;
            if (debugflag) syslog(LOG, "$GPRMC,%s\n", gpstxt);
            if (gpstxt[0] == ',') return (1);
            else {
                date = 0;
                for (cnt = 0; cnt < 62 && gpstxt[cnt] && !date; cnt++) {
                    if (gpstxt[cnt] == ',') dd++;
                    if (dd == 8) {
                        sscanf(gpstxt + cnt + 1, "%ld", &date);
                        if ((date % 100) < 6) return (1);
                        if (debugflag > 3) syslog(LOG, "DATE %06ld\n", date);
                    }
                }
                flag = 0;
                if (strstr(gpstxt, ",W,")) flag |= 2;
                if (strstr(gpstxt, ",S,")) flag |= 1;
                StripText(gpstxt);
                retv = sscanf(gpstxt, "%lf %lf %lf", &a, &lat, &lon);
                if (retv != 3) { lat = lon = 0; }
                if (retv > 0) {
                    gpstime = floor(a) * 100;
                    gpsdate = date;
                    lat /= 100.0;
                    a = floor(lat);
                    lat = (lat - a) * 100.0 / 60.0 + a;
                    lon /= 100.0;
                    a = floor(lon);
                    lon = (lon - a) * 100.0 / 60.0 + a;
                    rmcok = 1;
                    if (flag & 1) lat = -lat;
                    if (flag & 2) lon = -lon;

                    if (debugflag > 0)
                        syslog(LOG, "DATE %06ld TIME=%08ld LAT=%lf LON=%lf\n", gpsdate, gpstime, lat, lon);

                    // set time but do not set latlon
                    if (lat == 0 && lon == 0) return (2);
                } else return (1);
            }
        }
    }
    if (!SendMode(2)) {
        if (!GetLogData(gpstxt, 1500))
            return (0);
        else {

            if ((gpstxt[0] < '0' || '9' < gpstxt[0]) && gpstxt[0] != ',') {
                if (debugflag)
                    syslog(LOG, "Corrupt GGA data %c %d\n", gpstxt[0], gpstxt[0]);
                return (2);
            }
            if (debugflag) syslog(LOG, "$GPGGA,%s\n", gpstxt);

            if (gpstxt[0] == ',') return (2);
            else {
                pt = strstr(gpstxt, ",M");
                if (pt) {
                    pt--;
                    while (pt[0] != ',') pt--;
                    pt++;
                    sscanf(pt, "%lf", &gpsalt);
                    if (debugflag > 1) syslog(LOG, "ALT=%.1lf\n", gpsalt);
                }
                flag = 0;
                if (strstr(gpstxt, ",W,")) flag |= 2;
                if (strstr(gpstxt, ",S,")) flag |= 1;
                StripText(gpstxt);
                if (sscanf(gpstxt, "%lf %lf %lf", &a, &lat, &lon) == 3) {
                    if (lat == 0 && lon == 0) return (1);
                    gpstime = floor(a) * 100;
                    lat /= 100.0;
                    a = floor(lat);
                    lat = (lat - a) * 100.0 / 60.0 + a;
                    lon /= 100.0;
                    a = floor(lon);
                    lon = (lon - a) * 100.0 / 60.0 + a;
                    ggaok = 1;
                    if (flag & 1) lat = -lat;
                    if (flag & 2) lon = -lon;

                    if (debugflag > 0) syslog(LOG, "TIME=%08ld LAT=%lf LON=%lf\n", gpstime, lat, lon);
                } else return (1);
            }
        }
    }
    if (lat == 0.0 && lon == 0.0) return (1);
    gpslat = lat;
    gpslon = lon;
    if (rmcok == 0) {
        return (1);
    } else {
        return (2); // notify that GPStime&date is correct
    }
}

int GetGPSnew() {
    long dd, ggaok, rmcok, date;
    double a, lat, lon;
    long cnt;
    char *pt;
    int iter, retv, flag;
    char *gpstxt;

    gpstxt = (char *) sbuf;
    gpstxt[0] = 0;
    iter = 6;
    lat = lon = rmcok = ggaok = 0;

    GetGPSString("$GPRMC,", 1500, gpstxt, 1024);
    if (!gpstxt[0]) return (0);

    if ((gpstxt[0] < '0' || '9' < gpstxt[0]) && gpstxt[0] != ',') {
        if (debugflag) syslog(LOG, "Corrupt GPS data\n");
        return (0);
    }
    dd = 0;
    if (debugflag) syslog(LOG, "$GPRMC,%s\n", gpstxt);
    if (gpstxt[0] == ',') return (1);
    else {
        date = 0;
        for (cnt = 0; cnt < 62 && gpstxt[cnt] && !date; cnt++) {
            if (gpstxt[cnt] == ',') dd++;
            if (dd == 8) {
                sscanf(gpstxt + cnt + 1, "%ld", &date);
                if ((date % 100) < 6) return (1);
            }
        }
        flag = 0;
        if (strstr(gpstxt, ",W,")) flag |= 2;
        if (strstr(gpstxt, ",S,")) flag |= 1;
        StripText(gpstxt);
        retv = sscanf(gpstxt, "%lf %lf %lf", &a, &lat, &lon);
        if (retv != 3) { lat = lon = 0; }
        if (retv > 0) {
            gpstime = floor(a) * 100;
            gpsdate = date;
            lat /= 100.0;
            a = floor(lat);
            lat = (lat - a) * 100.0 / 60.0 + a;
            lon /= 100.0;
            a = floor(lon);
            lon = (lon - a) * 100.0 / 60.0 + a;
            rmcok = 1;
            if (flag & 1) lat = -lat;
            if (flag & 2) lon = -lon;

            if (debugflag > 0) syslog(LOG, "DATE %06ld TIME=%08ld LAT=%lf LON=%lf\n", gpsdate, gpstime, lat, lon);

            // set time but do not set latlon
            if (lat == 0 && lon == 0) return (2);
        } else return (1);
    }
    GetGPSString("$GPGGA,", 1500, gpstxt, 1024);
    if (!gpstxt[0]) return (0);

    if ((gpstxt[0] < '0' || '9' < gpstxt[0]) && gpstxt[0] != ',') {
        if (debugflag) syslog(LOG, "Corrupt GPS data\n");
        return (0);
    }
    if (debugflag) syslog(LOG, "$GPGGA,%s\n", gpstxt);

    if (gpstxt[0] == ',') return (1);
    else {
        pt = strstr(gpstxt, ",M");
        if (pt) {
            pt--;
            while (pt[0] != ',') pt--;
            pt++;
            sscanf(pt, "%lf", &gpsalt);
            if (debugflag > 0) syslog(LOG, "ALT=%.1lf\n", gpsalt);
        }
        flag = 0;
        if (strstr(gpstxt, ",W,")) flag |= 2;
        if (strstr(gpstxt, ",S,")) flag |= 1;
        StripText(gpstxt);
        if (sscanf(gpstxt, "%lf %lf %lf", &a, &lat, &lon) == 3) {
            if (lat == 0 && lon == 0) return (1);
            gpstime = floor(a) * 100;
            lat /= 100.0;
            a = floor(lat);
            lat = (lat - a) * 100.0 / 60.0 + a;
            lon /= 100.0;
            a = floor(lon);
            lon = (lon - a) * 100.0 / 60.0 + a;
            ggaok = 1;
            if (flag & 1) lat = -lat;
            if (flag & 2) lon = -lon;
        } else return (1);
    }
    if (lat == 0.0 && lon == 0.0) return (1);
    gpslat = lat;
    gpslon = lon;
    if (rmcok == 0) {
        return (1);
    } else {
        return (2); // notify that GPStime&date is correct
    }
}

long FileSize(char *name) {
    int f;
    long a;
    f = open(name, O_RDONLY);
    if (f == -1) return (-1);
    a = lseek(f, 0, SEEK_END);
    close(f);
    return (a);
}

void TouchFile(char *name) {
    FILE *f;
    if (FileSize(name) < 1) return;
    f = fopen(name, "r+b");
    if (f < (FILE *) 1) f = fopen(name, "w+b");
    if (f > (FILE *) 1) fclose(f);
}

int GetGPS() {
    long dd;
    int ret;
    struct tm t;
    gpsok = 0;
    ret = 0;

    if (pcbversion == 0) return;

    if (pcbversion > 3) {
        SetTaskPrio(TASKPRIO_HIGH);
        ret = GetGPS3();
        SetTaskPrio(TASKPRIO_LOW);
    }
    if (ret == 0)
        ret = GetGPSnew(gpsfile);

    if (ret < 2 || gpsdate == 0 || gpstime == 0) {
        // a return value below 2 indicates that time&date was not found
        GetCPUTime();
        return (ret);
    }
    gpsok = 1;
    StatusWriter(SETCPUTIME);
    t.tm_year = (gpsdate % 100) + 100;
    t.tm_mon = ((gpsdate / 100) % 100) - 1;
    t.tm_mday = ((gpsdate / 10000) % 100);

    // syslog(LOG,"YMD %d %d %d\n",t.tm_year , t.tm_mon , t.tm_mday);

    dd = gpstime / 100;
    t.tm_sec = (dd % 100);
    t.tm_min = ((dd / 100) % 100);
    t.tm_hour = ((dd / 10000) % 100);

    // syslog(LOG,"HMS %d %d %d\n",t.tm_hour , t.tm_min , t.tm_sec);

    time_t tt = mktime(&t);
    if (tt != 0)
        stime(&tt);

    ResyncTimer();
    timeisset = 1;
    TouchFile(uploadname);

    if (sun_getgpsfromfile) {
        FILE *f = fopen("gps.txt", "r");
        if (f > (FILE *) 0) {
            fscanf(f, "%lf %lf\n", &gpslat, &gpslon);
            fclose(f);
        }
    }
    return (1);
}

#define Execute system
#define Delete remove

void Reboot() {
    syslog(LOG, "Rebooting");
    system("reboot");
}

int KbdCheck() {
    FILE *f;
    if (realtime == 1) {
        if (FileSize(uploadname) <= 0 && MemoryFileSize() > 0) {
            SaveMemoryFile(uploadname);
        }
    }
    f = fopen(cmdname, "r");
    if (f) {
        StatusWriter(COMMAND);
        fgets(txt, sizeof(txt) - 1, f);
        do {
            if (strncmp(txt, "reboot", 6) == 0) {
                fclose(f);
                Delete(cmdname);
                Reboot();
            }
            if (strncmp(txt, "exit", 4) == 0) {
                fclose(f);
                Delete(cmdname);
                CleanUp(0);
            }
            if (strncmp(txt, "exec", 4) == 0) {
                char name[32];
                sscanf(txt + 4, "%s", name);
                StatusWriter(EXECUTE);
                syslog(LOG, "%s\n", name);
                Execute(name);
                goto donekbd;
            }
            if (strncmp(txt, "del", 3) == 0) {
                char name[32];
                sscanf(txt + 3, "%s", name);
                StatusWriter(DELETE);
                syslog(LOG, "%s\n", name);
                Delete(name);
                goto donekbd;
            }
            if (strncmp(txt, "pause", 5) == 0) {
                pause = 1;
                StatusWriter(PAUSE);
                goto donekbd;
            }
            if (strncmp(txt, "resume", 6) == 0) {
                pause = 0;
                StatusWriter(RESUME);
                goto donekbd;
            }
            if (strncmp(txt, "poweroff", 8) == 0) {
                SwitchRelay(0);
                goto donekbd;
            }
            if (strncmp(txt, "poweron", 7) == 0) {
                SwitchRelay(1);
                goto donekbd;
            }
            donekbd:
            txt[0] = 0;
            fgets(txt, sizeof(txt) - 1, f);
        } while (txt[0]);

        if (f > (FILE *) 0) {
            fclose(f);
            Delete(cmdname);
        }
    }

    return (0);
}

int DeleteOldest() {
    struct dirent *dir;
    DIR *d;

    int minr, minu, nr;
    char oldestname[20];
    char cmd[20];

    msleep(128);
    syslog(LOG, "DeleteOldest");
    DelOldestCnt++;
    minr = minu = 0x7fff;

    d = opendir(".");
    if (d != NULL) {
        while (dir = readdir(d)) {
            sscanf(&dir->d_name[1], "%x", &nr);

            if ((dir->d_name[0] == 'r') && dir->d_type == DT_DIR) {
                if (minr > nr) minr = nr;
            }
            if ((dir->d_type != DT_DIR) && ((dir->d_name[0] == 'u') && (strstr(dir->d_name, ".pak")))) {
                if (minu > nr) minu = nr;
            }
        }
        closedir(d);
    }

    if ((minr == 0x7fff) && (minu == 0x7fff)) {
        if (DelOldestCnt > 200) Reboot();
        return (0);
    }
    if (minr != 0x7fff) {
        sprintf(oldestname, "r%03x", minr);
        sprintf(cmd, "rm -f %s/*", oldestname);
        Execute(cmd);
        rmdir(oldestname);
        return (1);
    }
    sprintf(oldestname, "u%03x.pak", minu);
    syslog(LOG, "Deleting %s to free up space\n", oldestname);
    Delete(oldestname);
    return (1);
}

void ReadSerialNumber() {
    int i, j;
    FlushSerial(port_spec, 128);
    txt[0] = '?';
    txt[1] = 'x';
    txt[2] = 0;
    txt[3] = 0;
    WriteSerial(&txt, 4);

    j = 0;
    txt[0] = 0;
    while (CheckSerial(port_spec, 512)) {
        j += ReadSerial(port_spec, &txt[j], 256 - j);
    }
    for (i = 0; i < j; i++) {
        if ((txt[i] == 0x0d) || (txt[i] == 0x0a)) txt[i] = 0;
        if ((txt[i] == 0x06) || (txt[i] == 21)) txt[i] = ' ';
    }
    if (strlen(txt)) {
        strncpy(instrumentname, txt + 1, 15);
        syslog(LOG, "Successfully read spectrometer serial number '%s'\n", instrumentname);
    }
}


void ReadTemperature() {
    int j;
    short tmp;
    FlushSerial(port_spec, 128);
    txt[0] = '?';
    txt[1] = 't';
    WriteSerial(&txt, 2);

    j = 0;
    txt[0] = 0;
    while (CheckSerial(port_spec, 512)) {
        j += ReadSerial(port_spec, &tmp, 2 - j);
    }
    if (j == 2) {
        tmp = swp(tmp);
        temperature = tmp;
        temperature /= 100.0;
        syslog(LOG, "Temperature inside spectrometer: %.2f deg C\n", temperature);
    }
}


#include "algofornovac.c"

char cfgfile[16];
int cfgonce = 0;


void CheckSunAngle() {
    if ((gpslat == 0.0) && (gpslon == 0.0)) {
        return;
    }
    if (FileSize(cfgStratoName) > 0) {
        if (debugflag)
            syslog(LOG, "Stratosphere cfg found %s\n", cfgStratoName);
    } else {
        return;
    }

    if (GetSunAngle() < stratoAngle) {
        strcpy(cfgfile, cfgStratoName);
        ReadSettingFile(cfgfile);
    } else {
        strcpy(cfgfile, cfgtxtname);

    }

}

// the file cfgonce.txt contains measurements that should only be
// executed one time

void HandleCfgOnce() {
    if (cfgonce) {
        cfgonce = 0;
        Delete(cfgoncename);
        strcpy(cfgfile, cfgtxtname);
        ReadSettingFile(cfgfile);
    } else {
        if (FileSize(cfgoncename) > 0) {
            cfgonce = 1;
            strcpy(cfgfile, cfgtxtname);
            ReadSettingFile(cfgfile);
        }
    }
}

int ftpiter;

void CheckFtpIteration() {
    if (serverip) {
        if (ftpiter <= 0) {
            if (FileSize(uploadname) > 0) {
                if (ftpconnect() == 0) {
                    if (debugflag > 0) syslog(LOG, "Could not send file");
                } else {
                    Delete(uploadname);
                }
            }
            ftpiter = ftpiterscans;
        } else
            ftpiter--;
    }
}

void CheckRenamePakFiles() {
    long siz_ul, siz_work;

    siz_work = MemoryFileSize();
    if (siz_work <= 0) return;

    siz_ul = FileSize(uploadname);
    if (siz_ul == 0) Delete(uploadname);

    if (siz_ul <= 0) {
        SaveMemoryFile(uploadname);
        // Rename(workname,uploadname);
    } else {
        if (realtime != 2) {
            if (siz_ul > 0) {
                sprintf(txt, "u%03x.pak", uploadcnt);
                SaveMemoryFile(txt);
                uploadcnt++;
            }
        }
    }
    if (uploadcnt >= 512) Reboot();
}


void RenamePakFile(char *name) {
    int idx;

    if (FileSize(name) <= 0) return;
    if (debugflag > 0) syslog(LOG, "Found existing %s\n", name);
    for (idx = 0; idx < 100; idx++) {
        sprintf(txt, "upload%d.pak", idx);
        if (FileSize(txt) <= 0) {
            sprintf(txt, "cp %s upload%d.pak", name, idx);
            Execute(txt);
            Delete(name);
            return;
        }
        if (debugflag > 1) syslog(LOG, "%s exists\n", txt);
    }
}


int main(int argc, char *argv[]) {

    if (debugflag > 0) syslog(LOG, "Compiled " __DATE__  " " __TIME__ "\n");

    /* define PIO10=STEP1 and PIO1=STEP2 as output , set to low */

    pcbversion = gpsalt = gpslat = gpslon = 0;
    solenoid = 7;
    ftpiterscans = 10;
    ftpiter = 0;
    smem1 = (long *) 0;
    sbuf = (unsigned short *) 0;
    hostipaddr = 0;
    memset(instrumentname, 0, 16);
    memset(spectrometerType, 0, 10);

    SwitchRelay(0);

    KbdCheck();

    StatusWriter(STARTPROGRAM);

    smem1 = (long *) dosallocate(4 * (maxlen));
    sbuf = (unsigned short *) dosallocate(buffertbytes);
    if ((!smem1) || (!sbuf)) {
        StatusWriter(MEMERR);
        CleanUp(0);
    }

    ReadSettingFile(cfgtxtname);

    if (realtime != 2) {
    } else {
        RenamePakFile(uploadname);
//      RenamePakFile(workname);
    }
    InitSerial();

    SwitchRelay(1);
    FlushSerial(port_spec, 128);
    ResyncTimer();
    StatusWriter(WAIT4S);


    if (strstr(spectrometerType, "HR2000") ||
        strstr(spectrometerType, "USB2000+")) {
        ChangeBaudrate();
    } else {
        CheckSerial(port_spec, 4000);
    }
    if (InitCommunication()) Reboot();

    if (GetVersion() >= 4) CheckBattery();
    SendMotor(0);

    if (skipmotor != 1) {
        short dir, node;
        if ((meas[0].flag & 0x07) == 7) SendSolenoid(3);
        if (debugflag > 0) StatusWriter(HOMEMOTOR);
        for (node = 0; node < 2; node++) {
            if (stepsperround[node]) {
                dir = 1;
                if (stepsperround[node] < 0) dir = -1;
                HomeMotor(node, dir);
            }
        }
    }
    ReadSerialNumber();
    CheckBattery();
    //  if(pcbversion>2) GetCompass();

    spectrumcnt = measpt = 0;
    strcpy(cfgfile, cfgtxtname);
    GetGPS();

    while (!KbdCheck()) {

        if (ReadSettingFile(cfgfile)) {
            // if the program gets here then there were no cfgfile file to read.
            if (cfgonce) {
                // When arrrived here, we are in the cfgonce.txt mode and no cfgonce.txt was found
                // Then return to usual cfg.txt mode
                cfgonce = 0;
            } else {
                // When arrived here ,we are in cfg.txt mode and no cfg.txt file exists
                CheckBattery();
                //              if(pcbversion>2) GetCompass();
                msleep(200);
            }
            strcpy(cfgfile, cfgtxtname);

        } else {
            // when arrived here, the cfg-file, (either cfg.txt or cfgonce.txt) was read successfully
            if (measpt >= measurecnt) {
                // when arrived here, we have just executed the last MEAS=
                // Now start from the beginning of the MEAS= list
                spectrumcnt = measpt = 0;
                CheckBattery();
                GetGPS();
                //              if(pcbversion>2) GetCompass();
            }
            if (measpt == 0) {
                // when arrived here, we will just start the MEAS= list from the beginning

                if ((strstr(spectrometerType, "HR4000")) ||
                    (strstr(spectrometerType, "USB4000")))
                    ReadTemperature();
                CheckSunAngle();
                HandleCfgOnce();
                CheckRenamePakFiles();
                CheckFtpIteration();
            }
            if (pause || (measurecnt == 0))
                msleep(1000);
            else {
                // when arrived here, we are not pausing
                if (meas[measpt].intsum == 0 && meas[measpt].extsum == 0) {
                    if (meas[measpt].pos) HomeMotor(0, meas[measpt].pos);
                    if (meas[measpt].pos2) HomeMotor(1, meas[measpt].pos2);
                } else {
                    if (debugflag) syslog(LOG, "Measuring %s (%d)\n", meas[measpt].name, measpt);
                    lastpos[0] = motorposition[0];
                    lastpos[1] = motorposition[1];
                    MoveMotor(meas[measpt].pos, meas[measpt].pos2);


                    initspectrometer(meas[measpt].chn, meas[measpt].intsum);

                    SetTaskPrio(TASKPRIO_HIGH);
                    // raise priority so we can make fast wind measurements
                    for (repeat = 0; repeat < meas[measpt].repetitions && !KbdCheck(); repeat++) {
                        if ((meas[measpt].flag & 0x08) && repeat) {
                            if (repeat & 1) MoveMotor(lastpos[0], lastpos[1]);
                            else MoveMotor(meas[measpt].pos, meas[measpt].pos2);
                        }
                        if (!doscan()) {}
                        else {
                            // something went wrong, reinitialize and try again
                            SetTaskPrio(TASKPRIO_LOW);
                            // lower priority to allow other tasks to get time
                            ResetSpectrometer();
                            initspectrometer(meas[measpt].chn, meas[measpt].intsum);
                            SetTaskPrio(TASKPRIO_HIGH);
                            // raise priority so we can make fast wind measurements
                        }
                    }
                    SetTaskPrio(TASKPRIO_LOW);
                    // lower priority to allow other tasks to get time
                    SaveScan();
                    spectrumcnt++;
                }
                ++measpt;
            }
        }
    }
    return (0);
}


void CleanUp(int retV) {

    SwitchRelay(0);

    if (smem1) free(smem1);
    if (sbuf) free(sbuf);

    if (port_motor1) close(port_motor1);
    if (port_motor2) close(port_motor2);
    if (port_spec) close(port_spec);

    if (fdB) close(fdB);
    if (fdC) close(fdC);
    if (fdD) close(fdD);
    StatusWriter(STOPPROGRAM);

    exit(retV);
}

