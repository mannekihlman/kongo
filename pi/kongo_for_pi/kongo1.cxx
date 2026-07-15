#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Compiler cannot find C++ library. Solve by avioding string
// #include <string>

#include <math.h>
#include <dirent.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <netdb.h>
#include <syslog.h>
#include <unistd.h>

#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/stat.h>

typedef unsigned long u32;
typedef unsigned short u16;
typedef unsigned char u8;

namespace {
    //----------------------------------------------
    // Function forward declarations

    long FileSize(const char *const name);

    void ReadSerialNumber();

    int DeleteOldest();

    int VerboseDeleteOldest();

    int KbdCheck();

    int GetSwitchStatus(int node);

    void CleanUp(int);

    void msleep(u16 t);

    int CheckSerial(int port, long t);

    void PrepareAvantesSpectrometer();

    int AvantesAddScan(short isum, short chn, long sleepsum);

    int AvantesInitspectrometer(short chn, short sumcnt);

    float AvantesGetTemperature();

    int ReadSerial(int port, void *pt, long len);

    //----------------------------------------------

    char PortNameSpectrometer[256] = "/dev/ttymxc1";
    char PortNameADC[256] = "/dev/ttymxc2";
    char PortNameGPS[256] = "/dev/ttyUSB0";

    char cfgMainName[] = "cfgmain.txt";
    char cfgStratoName[] = "cfgstrat.txt";
    char cfgtxtname[] = "cfg.txt";
    char cfgoncename[] = "cfgonce.txt";

    char uploadname[] = "upload.pak";
    char statusname[] = "/var/status.dat";
    char cmdname[] = "command.txt";

    int LOG = (LOG_USER | LOG_DEBUG);
    int debugflag = 2;

    int maxIntTime = 5000;
    int minIntTime = 10;
    int stratoAngle = 5;

    u32 hostipaddr;
    char instrumentname[32];
    char spectrometerType[16];
    char hhmmss_time[8];
    long starttime, stoptime, gpstime, gpsdate, yymmdd_date;
    int gpsok = 0;
    u8 timeisset = 0;

    u32 serverip = 0;
    char username[16];
    char password[16];
    int ftpiterscans;
    char txt[1024];

    int uploadcnt;
    float compassdir = 0;
    float tiltX = 0;
    float tiltY = 0;
    float temperature = 0.0;
    int powersave = 0;
    long lastidx = 0;
    int skipmotor = 0;
    int verboseFileName = 0;
    int startchn = 0;
    int stopchn = 0;
    float batterylimit = 11.0;
    float battery = 0.0;
    int motorswap = 0;

    int motorstepscmp[2] = {0, 0};
    long motorposition[2] = {0, 0};
    long lastpos[2] = {0, 0};
    short saved_viewangle[2];
    short saved_pos[2];

    int microsteps = 16;    // The Trinamic TMCM-1070 default uses 16 microsteps
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
    int maxv_idx = 0;
    int paused = 0;
    char coneangle = 90;
    double minElevAngle = 20;

#define maxlen 4096
#define buffertbytes 20000
    long *smem1 = 0;
    u16 *sbuf;
    short speclen, repeat;
    short solenoid = 0;
    int sun_getgpsfromfile = 0;
    u8 motor1_on = 0;
    u8 motor2_on = 0;

    int port_spec = 0;
    int port_adc = 0;

    long specbaud = 115200;
    int DelOldestCnt = 0;

#include "memoryfile.c"
#include "taskit.c"
#include "status.h"
#include "movepakfiles.c"

//----------------------------------------------
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
    u8 flag;
};
struct measurementStr meas[maxmeas];


//----------------------------------------------
#define TASKPRIO_HIGH -20
#define TASKPRIO_LOW 10

void SetTaskPrio(short prio) {
    setpriority(PRIO_PROCESS, 0, prio);
}

//----------------------------------------------
void msleep(u16 t) {
    if (t < 1) {
        return;
    }
    usleep(t * 1000);
}

//----------------------------------------------
int CheckSerial(int port, long t) {
    if (debugflag > 3) syslog(LOG, "CheckSerial\n");

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(port, &rfds);

    tv.tv_usec = t * 1000;
    tv.tv_sec = 0;
    while (tv.tv_usec >= 1000000) {
        tv.tv_usec -= 1000000;
        tv.tv_sec++;
    }
    return select(port + 1, &rfds, NULL, NULL, &tv);
}


//----------------------------------------------
void WriteSerial(const void *pt, u16 l) {
    write(port_spec, pt, l);
}

//----------------------------------------------
int ReadSerial(int port, void *pt, long len) {
    if (debugflag > 3) syslog(LOG, "ReadSerial");
    int i = read(port, pt, len);
    if (i < 0) i = 0;
    return (i);
}


//----------------------------------------------
void FlushSerial(int port, long t) {
    char txt;
    while (CheckSerial(port, t)) {
        ReadSerial(port, &txt, 1);
        if (debugflag > 2) syslog(LOG, "0x%0x\n", txt);
    }
}

//----------------------------------------------
void StatusWriter(u8 s) {
    FILE *f;
    u8 *status;
    u16 last, first, *p;
    if (debugflag) StatusPrint(s);

    status = (u8 *) txt;
    p = (u16 *) status;
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


//----------------------------------------------
int OpenSerialPort(const char *const name, int speed) {
    struct termios tm;

    int port = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (port == -1) {
        if (debugflag)
            syslog(LOG, "Could not open port '%s'\n", name);
        StatusWriter(SERIALOPENERR);
        return 0U;
    }

    // This is axiomtek specific
    int type = 1;       // RS232
#define SET_COM_TYPE        0x542A
    ioctl(port, SET_COM_TYPE, &type);

    cfmakeraw(&tm);
    cfsetispeed(&tm, speed);
    cfsetospeed(&tm, speed);
    tm.c_cflag = (speed | CS8 | CREAD | CLOCAL | HUPCL);
    tm.c_lflag &= ~(ECHO);
    tm.c_oflag = 0;
    tm.c_iflag = 0;
    tm.c_lflag = 0;
    tcsetattr(port, TCSANOW, &tm);
    return port;
}

//----------------------------------------------
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

//----------------------------------------------
void setdirection(short node, short dir) {
    if (sun_getgpsfromfile) { // we should not interfere with the solartracker
        return;
    }

    if (dir > 0) motorposition[node]++;
    else motorposition[node]--;
}

//----------------------------------------------
void InitSerial() {
    port_spec = OpenSerialPort(PortNameSpectrometer, BaudToBits(specbaud));
    port_adc = OpenSerialPort(PortNameADC, B115200);
    Taskit::InitADC();

    if (debugflag > 0) StatusWriter(SERIALOPEN);
}

//----------------------------------------------
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

//----------------------------------------------
void SetASCII() {
    if (debugflag) syslog(LOG, "Setting spectrometer to ascii mode\n");

    int retry = 10;
    do {
        retry--;
        WriteSerial("aA", 2);
        char LF[] = {0x0d};
        WriteSerial(LF, 1);

        txt[0] = 0;
        while (CheckSerial(port_spec, 128)) {
            ReadSerial(port_spec, &txt[0], 1);
        }

    } while ((txt[0] != 32) && retry);

}

//----------------------------------------------
void SetBinary() {
    if (debugflag) syslog(LOG, "Setting spectrometer to binary mode\n");
    txt[0] = 'b';
    txt[1] = 'B';
    WriteSerial(txt, 2);
    FlushSerial(port_spec, 128);
}

//----------------------------------------------
int isAvantesSpectrometer() {
    if (strstr(spectrometerType, "AVANTES")) {
        return 1;
    } else {
        return 0;
    }
}

//----------------------------------------------
int isMayaproSpectrometer() {
    if (strstr(spectrometerType, "MAYAPRO")) {
        return 1;
    } else {
        return 0;
    }
}


//----------------------------------------------
int isHR2000Spectrometer() {
    if (strstr(spectrometerType, "HR2000")) {
        return 1;
    } else {
        return 0;
    }
}

//----------------------------------------------
int isHR4000Spectrometer() {
    if (strstr(spectrometerType, "HR4000")) {
        return 1;
    } else {
        return 0;
    }
}

//----------------------------------------------
int isUSB4000Spectrometer() {
    if (strstr(spectrometerType, "USB4000")) {
        return 1;
    } else {
        return 0;
    }
}

//----------------------------------------------
int isUSB2000PlusSpectrometer() {
    if (strstr(spectrometerType, "USB2000+")) {
        return 1;
    } else {
        return 0;
    }
}

//----------------------------------------------
void ResetSpectrometer() {
    if (isAvantesSpectrometer()) {
        PrepareAvantesSpectrometer();
        return;
    }

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
        return;
    }

    txt[0] = 'Q';
    WriteSerial(&txt, 1);
    if (CheckSerial(port_spec, 128)) ReadSerial(port_spec, txt, 1);
    if ((txt[0] == 0x06) || (txt[0] == 21)) {
        SetBinary();
        return;
    }

    // spectrometer does not respond but send binary anyway just in case

    SetBinary();
}

//----------------------------------------------
u16 swp(u16 in) {
    // This code converts a short between little-endian and big-endian and vice versa
    u8 *p1, *p2;
    u16 ut;
    p1 = (u8 *) &ut;
    p2 = (u8 *) &in;
    ut = p2[1];
    ut |= p2[0] << 8;
    return (ut);
}

//----------------------------------------------
u32 longswap(u8 *p2) {
    // This code converts a longword between little-endian and big-endian and vice versa
    u32 ut;
    ut = p2[0];
    ut = ut << 8;
    ut |= p2[1];
    ut = ut << 8;
    ut |= p2[2];
    ut = ut << 8;
    ut |= p2[3];
    return (ut);
}

//----------------------------------------------
void StripText(char *outtxt) {
    unsigned int i;
    for (i = 0; i < strlen(outtxt); i++) {
        if ((outtxt[i] < '0' || outtxt[i] > '9') && outtxt[i] != '.') outtxt[i] = ' ';
    }
}

//----------------------------------------------

#include "mk_compress.c"

//----------------------------------------------
void WriteMKZY(int speclen, long *spec, short channel) {
    long last, tmp, *pt;
    int i;
    u16 outsiz;
    u32 checksum;
    u16 *p;

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
    p = (u16 *) &checksum;
    MKZY.checksum = p[0] + p[1];

    memset(sbuf, 0, buffertbytes);

    if (debugflag > 3) syslog(LOG, "mk_compress called");
    outsiz = mk_compress(spec, (u8 *) sbuf, speclen);

    if (debugflag > 3) syslog(LOG, "mk_compress done");
    MKZY.ident[0] = 'M';
    MKZY.ident[1] = 'K';
    MKZY.ident[2] = 'Z';
    MKZY.ident[3] = 'Y';
    strncpy(MKZY.name, meas[measpt].name, 12);
    strncpy(MKZY.instrumentname, instrumentname, 16);
    MKZY.hdrsize = 114;
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


    u8 buffer[114];
    memcpy(buffer, &MKZY, 114 - 16);
    memcpy(&buffer[114 - 16], &MKZY.ADC[0], 16);
    if (mwrite((char *) buffer, 114, (char *) sbuf, outsiz) == 0) {
        if (debugflag) {
            syslog(LOG, "Not enough RAM memory to store work.pak file!\nYou must reduce nr of measrements in cfg.txt");
        }
    }
}

//----------------------------------------------
void SaveData(char *p, long len) {
    FILE *f;
    f = fopen("dbg.dat", "wb");
    if (f < (FILE *) 1) return;
    fwrite(p, len, 1, f);
    fclose(f);
}

int dosave = 0;

//----------------------------------------------
void SaveScan() {
    if (debugflag > 2) syslog(LOG, "SaveScan called");
    if (dosave == 0) return;
    if (debugflag > 1)
        syslog(LOG, "Date: %06ld Stoptime: %08ld\n", gpsdate, stoptime);

    WriteMKZY(speclen, smem1, meas[measpt].chn);

    dosave = 0;
    if (debugflag > 2) syslog(LOG, "SaveScan OK");
}

//----------------------------------------------
void GetCPUTime() {
    if (debugflag > 8) syslog(LOG, "GetCPUTime 0");
    struct timeval tv;

    gettimeofday(&tv, 0);
    struct tm *tm = localtime(&tv.tv_sec);

    gpstime = tm->tm_hour;
    gpstime = gpstime * 100 + tm->tm_min;
    gpstime = gpstime * 100 + tm->tm_sec;
    sprintf(hhmmss_time, "%06d", gpstime);

    gpsdate = tm->tm_mday;
    gpsdate = gpsdate * 100 + tm->tm_mon + 1;
    gpsdate = gpsdate * 100 + ((tm->tm_year + 100) % 100);

    yymmdd_date = ((tm->tm_year + 100) % 100);
    yymmdd_date = yymmdd_date * 100 + tm->tm_mon + 1;
    yymmdd_date = yymmdd_date * 100 + tm->tm_mday;
}

#include "ftpclient.c"

//----------------------------------------------
void handleMayaPro(int bytes) {
    int j;
    u16 offs, mode32bit;
    u32 a1;
    mode32bit = 0;

    offs = 6;
    // on the new spectrometers this value indicates that 32bit data follows.
    if (debugflag > 1) {
        syslog(LOG, "sbuf= %d %d %d %d %d %d\n", sbuf[0], sbuf[1], sbuf[2], sbuf[3], sbuf[4], sbuf[5]);
    }
    if (sbuf[1]) {
        mode32bit = 1;
        offs = 7;
    }

    speclen = bytes >> 1;
    speclen--;  // decrease one representing the last word (should be 0xfffd)
    if (swp(sbuf[speclen]) != 0xfffd) {
        if (debugflag) syslog(LOG, "Last word of transmission is incorrect: 0x%4x\n", sbuf[speclen]);
        return;
    }

    // Does the header have pixelstart and pixelstop information
    if (startchn != stopchn) {
        if (debugflag > 1)
            syslog(LOG, "Spectrum has pixelstart info\n");

        if (mode32bit) {
            offs += 1;
            speclen -= offs;
        } else {
            offs += 2;
            speclen -= offs;
        }
    } else {
        speclen -= offs;
    }

    maxv = 0;
    if (mode32bit == 0) {
        u16 *pS;
        pS = (u16 *) (&sbuf[offs]);

        if (speclen > maxlen) speclen = maxlen;
        for (j = 0; j < speclen; j++) {
            a1 = swp(pS[j]);
            smem1[j] += a1;

            if ((maxv < a1)) {
                maxv_idx = j;
                maxv = a1;
            }

        }
    } else {
        u32 *pS;
        if (debugflag > 1) syslog(LOG, "Spectrum is 32 bit precision!");
        speclen = speclen >> 1;

        // source pointer
        pS = (u32 *) (&sbuf[offs]);

        if (debugflag > 10)
            SaveData((char *) pS, speclen * 4);

        if (speclen > maxlen) speclen = maxlen;
        for (j = 0; j < speclen; j++) {
            u8 *b = (u8 *) &pS[j];
            a1 = longswap(b);
            smem1[j] += a1;

            if ((maxv < a1)) {
                maxv_idx = j;
                maxv = a1;
            }
        }
    }
}

//----------------------------------------------
int AddScan(short isum, short chn, long sleepsum) {
    char *bptr;
    char ck[1];
    u16 offs, mode32bit;
    u32 a1;
    int i, j;

    do {
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

        if (CheckSerial(port_spec, 2560)) ReadSerial(port_spec, ck, 1);

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
        while (sleepsum >= 0) {
            if (debugflag > 1) syslog(LOG, "sleepsum=%d\n", sleepsum);
            if (CheckSerial(port_spec, 32767)) break;
            sleepsum -= 32767;
        }

        i = 0;
        bptr[i] = 0;

        while ((CheckSerial(port_spec, 400)) && (i < buffertbytes)) {
            j = ReadSerial(port_spec, &bptr[i], buffertbytes - i);
            if (j > 0) {
                i += j;
            }
        }
    } while (bptr[0] == 253);

    if (i < 16) {
        StatusWriter(DOSCANTIMEOUT);
        if (debugflag) {
            u32 tmp = sbuf[j];
            syslog(LOG, "i=%d\n", i);
            for (j = 0; j < i; j++) syslog(LOG, "%d %0x%04x\n", j, tmp);
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
    if (debugflag > 1) { syslog(LOG, "Got %d bytes\n", i); }

    mode32bit = 0;

    offs = 7; // Default header size is 7

    if (isMayaproSpectrometer()) {
        handleMayaPro(i);
    } else {
        if (isHR4000Spectrometer() || isUSB4000Spectrometer() ||
            isUSB2000PlusSpectrometer() || isMayaproSpectrometer()) {
            // on the new spectrometers this value indicates that 32bit data follows.
            if (sbuf[1]) {
                mode32bit = 1;
            } else {
                // the offset is really a mess. these spectrometers
                // have a different header than the rest if 16 bits
                offs = 8;
            }

            // on the old spectrometers it contains the number of the ADC channel
            // so the correct setting of SPECTROMETERTYPE= is important

            if (debugflag > 1) { for (j = 1; j < offs; j++) syslog(LOG, "%d %d\n", j, swp(sbuf[j])); }

        }

        speclen = i >> 1;
        speclen--;  // decrease one representing the last word (should be 0xfffd)
        if (swp(sbuf[speclen]) != 0xfffd) {
            if (debugflag) syslog(LOG, "Last word of transmission is incorrect: 0x%4x\n", sbuf[speclen]);
            return (1);
        }

        // Does the header have pixelstart and pixelstop information
        if (sbuf[6]) {
            if (debugflag > 1)
                syslog(LOG, "Spectrum has pixelstart info\n");

            {
                if (mode32bit) {
                    offs = 13;
                } else {
                    offs = 10;
                }
                speclen -= offs;
            }
        } else {
            speclen -= offs;
        }

        maxv = 0;
        if (mode32bit == 0) {
            u16 *pS;
            pS = (u16 *) (&sbuf[offs]);

            if (speclen > maxlen) speclen = maxlen;
            for (j = 0; j < speclen; j++) {
                a1 = swp(pS[j]);
                smem1[j] += a1;
                if ((maxv < a1)) {
                    maxv_idx = j;
                    maxv = a1;
                }

            }
        } else {
            u32 *pS;
            if (debugflag > 1) syslog(LOG, "Spectrum is 32 bit precision!");
            speclen = speclen >> 1;

            // source pointer
            pS = (u32 *) (&sbuf[offs + 1]);

            if (debugflag > 10)
                SaveData((char *) pS, speclen * 4);

            if (speclen > maxlen) speclen = maxlen;
            for (j = 0; j < speclen; j++) {
                u8 *b = (u8 *) &pS[j];
                a1 = longswap(b);
                smem1[j] += a1;
                if ((maxv < a1)) {
                    maxv_idx = j;
                    maxv = a1;
                }
            }
        }
    }
    if (debugflag > 1) syslog(LOG, "AddScan successful");
    return (0);
}

//----------------------------------------------
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

//----------------------------------------------
void SendExposureTime(u16 inttime) {
    char t[10];
    char *p;
    int j, usenewcode;
    if (lastexposuretime == inttime) return;
    p = (char *) &inttime;

    usenewcode = 1;

    if (isHR4000Spectrometer() || isUSB4000Spectrometer()) {
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

            if (isHR4000Spectrometer()) {
                // Ok for short integration times. Christoph
                if (linttime < 650000) msleep(1000);
                else if (linttime < 1800000) msleep(10000);
                else msleep(30000);
                /* It can take up to 30 seconds to change
		   the exposure time using "i"! Christoph */
            } else {
                msleep(inttime);
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

            // Ok for short integration times. Christoph
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

//----------------------------------------------
void SendSumCnt(short sumcnt) {
    u8 *p;
    int j;

    if (lastsumcnt == sumcnt) return;
    p = (u8 *) &sumcnt;
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

//----------------------------------------------
void SendChn(short chn) {
    u8 *p;

    if (lastchn == chn) return;
    p = (u8 *) &chn;

    txt[0] = 'H';
    txt[1] = p[1];
    txt[2] = p[0];
    WriteSerial(txt, 3);
    CheckSerial(port_spec, 128);
    FlushSerial(port_spec, 1);
    lastchn = chn;
}

//----------------------------------------------
void SendPixelBox() {
    u16 sbuf;
    if ((laststartchn == startchn) && (laststopchn == stopchn)) return;


    if (startchn != stopchn) {
        u8 *p;

        if (debugflag) syslog(LOG, "Startchn=%d stopchn=%d\n", startchn, stopchn);
        p = (u8 *) &sbuf;

        if (isMayaproSpectrometer()) {
            SetASCII();
            char LF[] = {0x0d, 0};
            sprintf(txt, "P3");
            WriteSerial(txt, 3);
            FlushSerial(port_spec, 128);
            WriteSerial(LF, 1);
            FlushSerial(port_spec, 128);
            sprintf(txt, "%d", startchn);
            WriteSerial(txt, strlen(txt));
            FlushSerial(port_spec, 128);
            WriteSerial(LF, 1);
            FlushSerial(port_spec, 128);
            sprintf(txt, "%d", stopchn);
            WriteSerial(txt, strlen(txt));
            FlushSerial(port_spec, 128);
            WriteSerial(LF, 1);
            FlushSerial(port_spec, 128);
            sprintf(txt, "1");
            WriteSerial(txt, strlen(txt));
            FlushSerial(port_spec, 128);
            WriteSerial(LF, 1);
            FlushSerial(port_spec, 128);
            SetBinary();
        } else {
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
        }
        FlushSerial(port_spec, 128);
        laststartchn = startchn;
        laststopchn = stopchn;
    }
}

//----------------------------------------------
int initspectrometer(short chn, short sumcnt) {
    long inttime;
    long m, digitalnoise, maxcounts;
    short stest, ltest;
    float a, b, c, d;

    if (debugflag > 0)
        StatusWriter(INITSPEC);

    if (isAvantesSpectrometer()) {
        return AvantesInitspectrometer(chn, sumcnt);
    }

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
        if (isHR4000Spectrometer()) {
            stest = 10;
            ltest = 300;
            maxcounts = 16384;
            SendExposureTime(ltest);

            AddScan(0, chn, ltest);
            /* Throw the first scan away
	       I added this because I was sometimes getting too much
	       noise in this first spectrum. Now I throw it out. Christoph */
        } else if (isUSB4000Spectrometer() ||
                   isUSB2000PlusSpectrometer()) {
            stest = 24;
            ltest = 300;
            maxcounts = 65536;
            SendExposureTime(ltest);

            AddScan(0, chn, ltest);
            /* Throw the first scan away
	       I added this because I was sometimes getting too much
	       noise in this first spectrum. Now I throw it out. Christoph */
        } else if (isMayaproSpectrometer()) {
            stest = 10;
            ltest = 100;
            maxcounts = 65536;
            SendExposureTime(ltest);
            AddScan(0, chn, ltest);
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
            AddScan(0, chn, stest);
            if (pchannel != -1) {
                maxv = AvgChannels(smem1, pchannel, 10, maxlen);
            }
            digitalnoise = maxv;
            if (debugflag > 0)syslog(LOG, " Maxvalue: %d\n", digitalnoise);

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

int rotator[2] = {0, 0};
u16 lastPowerOn = 0;

//----------------------------------------------
int sendPower(int power, int m1, int m2) {
    u8 activate_mask = 0U;
    u8 deactivate_mask = 0U;

    if (debugflag) {
        syslog(LOG, "sendPower %d %d %d", power, m1, m2);
    }

    if ((power != 0) && ((m1 == 3) || (m2 == 3))) {
        // turn on motors
        activate_mask = 1 << 4;
    } else {
        // turn off motors
        deactivate_mask = 1 << 4;
    }
    if (Taskit::ControlOutput(activate_mask, deactivate_mask) == 0) {
        syslog(LOG, "SendPower %d %d %d failed", power, m1, m2);
    }
    msleep(100);
}

//----------------------------------------------
void set_motor1(int m1) {
    // If this function is called then motor should be turned on
    if (motor1_on != 3) {
        motor1_on = 3;
        sendPower(lastPowerOn, motor1_on, motor2_on);

    }
}

//----------------------------------------------
void set_motor2(int m2) {
    // If this function is called then motor should be turned on
    if (motor2_on != 3) {
        motor2_on = 3;
        sendPower(lastPowerOn, motor1_on, motor2_on);
    }
}

//----------------------------------------------
void getADC(u8 retries, u16 *adout) {
    if (debugflag > 2) {
        syslog(LOG, "getADC retries=%d\n", retries);
    }
    while (retries) {

        if (Taskit::GetAllADCchannels(adout)) {
            battery = 10.0 * adout[3] * 2.5 / 65536.0;

            if (isAvantesSpectrometer()) {
                temperature = AvantesGetTemperature();
            } else {
                const float t_mV = 2500.0 * adout[0] / 65536.0;
                temperature = (t_mV - 400.0) / 19.5;
            }
            if (debugflag > 0) {
                syslog(LOG, "Battery: %.2lf V\n", battery);
                syslog(LOG, "Temp: %.2lf deg C\n", temperature);
            }
            adout[0] = battery * 100.0;   // MKI inserted 211019
            return;
        }
        retries--;
    }
    syslog(LOG, "getADC failed\n");
}

//----------------------------------------------
void SendMotor(int power) {
    if (stepsperround[0]) {
        if (power == 0) motor1_on = 0;
        else motor1_on = 3;
    }
    if (stepsperround[1]) {
        if (power == 0) motor2_on = 0;
        else motor2_on = 3;
    }
    sendPower(lastPowerOn, motor1_on, motor2_on);
}

//----------------------------------------------
void SwitchRelay(u16 powerOn) {
    if (powerOn) {
        StatusWriter(POWERON);
    } else {
        StatusWriter(POWEROFF);
    }
    lastPowerOn = powerOn;
    syslog(LOG, "SwitchRelay %d", powerOn);
    sendPower(lastPowerOn, motor1_on, motor2_on);
}

//----------------------------------------------
int doscan() {
    short isum;
    long t;
    for (isum = 0; isum < meas[measpt].extsum && (!KbdCheck()); isum++) {
        if (debugflag > 1) syslog(LOG, "isum=%d", isum);
        t = meas[measpt].realexptime;
        t *= meas[measpt].intsum;

        int failure = 0;
        if (isAvantesSpectrometer()) {
            failure = AvantesAddScan(isum, meas[measpt].chn, t);
        } else {
            failure = AddScan(isum, meas[measpt].chn, t);
        }
        if (failure) {
            StatusWriter(SPECREFUSE);
            FlushSerial(port_spec, 100);
            return (1);
        }
        if (debugflag) syslog(LOG, "Got %d values. Maxvalues: %g (on index %d)", speclen, maxv, maxv_idx);
    }
    GetCPUTime();
    stoptime = gpstime;
    dosave = 1;
    if (debugflag > 1) StatusWriter(DOSCANDONE);

    saved_viewangle[0] = saved_pos[0] * 360.0 / abs(stepsperround[0]);
    if (stepsperround[1]) saved_viewangle[1] = saved_pos[1] * 360.0 / abs(stepsperround[1]);
    else saved_viewangle[1] = 0;

    return (0);     // Return 0 on success
}

//----------------------------------------------
void CheckBattery() {
    int batteryfail = 0;
    do {
        usleep(50000);
        getADC(6, &MKZY.ADC[0]);
        if (battery == 0.0) {
            if (debugflag) syslog(LOG, "No ADC installed");
            batteryfail = 0;
        }
        else {
            if (battery < batterylimit) {
                if (batteryfail == 0) {
                    batteryfail = 1;
                    if (debugflag) syslog(LOG, "Battery is low. Turning OFF\n");
                    SwitchRelay(0);
                }
                msleep(5000);
            } else {
                if (batteryfail == 1) {
                    batteryfail = 0;
                    if (debugflag) syslog(LOG, "Battery recovered. Turning ON\n");
                    SwitchRelay(1);
                }
            }
        }
    } while (batteryfail);
}

//----------------------------------------------
void dopulses(short node0, short node1) {
    struct timeval tv1, tv2;
    gettimeofday(&tv1, 0);

    u8 mask = 0;
    u8 dirActivate = 0;
    u8 dirDeactivate = 0x03;
    if (node0 != 0) {
        mask |= 1 << 6;
        if (node0 < 0) {
            dirActivate |= 0x01;
            dirDeactivate &= ~(0x01);
        }
    }
    if (node1 != 0) {
        mask |= 1 << 2;
        if (node1 < 0) {
            dirActivate |= 0x02;
            dirDeactivate &= ~(0x02);
        }
    }
    for (int i = 0; i < microsteps; i++) {
        Taskit::ControlOutput(mask | dirActivate, dirDeactivate);
        Taskit::ControlOutput(dirActivate, mask | dirDeactivate);
    }
    gettimeofday(&tv2, 0);

    long additionalWait = m_delay;
    additionalWait -= (tv2.tv_usec - tv1.tv_usec) / 1000;
    additionalWait -= (tv2.tv_sec - tv1.tv_sec) * 1000;
    if (debugflag > 1) {
        syslog(LOG, "dopulses %d %d (wait %d)", node0, node1, additionalWait);
    }
    if (additionalWait > 0) {
        msleep(m_delay);
    }
}

//----------------------------------------------
void sendsteps(long *steps) {
    short node, nodecnt;

    if (skipmotor == 2) return;

    for (nodecnt = 0; nodecnt < 2; nodecnt++) {
        if (steps[nodecnt]) {
            if (steps[nodecnt] == 1) rotator[nodecnt]++;
            else rotator[nodecnt]--;
            node = nodecnt;
            if (motorswap) { if (nodecnt) node = 0; else node = 1; }

            if (node == 0) set_motor1(rotator[0]);
            if (node == 1) set_motor2(rotator[1]);
        }
    }
}

//----------------------------------------------
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
    if (powersave) SendMotor(0);
    return;
}

//----------------------------------------------
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
                }
            }
        }
        if (domove) {
            dopulses(steps[0], steps[1]);
            sendsteps(steps);
        }
    } while (domove);
    if (debugflag > 1) StatusWriter(MOVEMOTOROK);
    if (powersave) SendMotor(0);

    return (0);
}

//----------------------------------------------
int ReadSettingFile(char *filename) {
    char *pt;
    char temp[30];

    FILE *fil = fopen(filename, "r");
    if (fil == 0) {
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
            if (pt = strstr(txt, "PortNameADC=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s", temp);
                strncpy(PortNameADC, temp, sizeof(PortNameADC));
            }
            if (pt = strstr(txt, "PortNameSpec=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s", temp);
                strncpy(PortNameSpectrometer, temp, sizeof(PortNameSpectrometer));
            }
            if (pt = strstr(txt, "PortNameGPS=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%s", temp);
                strncpy(PortNameGPS, temp, sizeof(PortNameGPS));
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
            if (pt = strstr(txt, "MINELEVANGLE=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%lf", &minElevAngle);
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
            if (pt = strstr(txt, "MICROSTEPS=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &microsteps);
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
                        int flag = 7;
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
                    syslog(LOG, "COMPASS=%.f %.f %.f %.f\n", compassdir * 0.1, tiltX * 0.1, tiltY * 0.1);
            }
            if (pt = strstr(txt, "SKIPMOTOR=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &skipmotor);
            }
            if (pt = strstr(txt, "VERBOSEFILENAME=")) {
                pt = strstr(txt, "=");
                sscanf(&pt[1], "%d", &verboseFileName);
            }
        }
    }
    fclose(fil);

    return (0);
}

//----------------------------------------------
int GetSwitchStatus(int node) {
    if (motorswap) { if (node) node = 0; else node = 1; }
    int state = 1;   // Input is active low

    for (int i = 0; (i < 10) && state; i++) {
        if (node == 0) {
            if (Taskit::GetInputs() & (1U << 5)) {
                state = 0;
            }
        }
        if (node == 1) {
            if (Taskit::GetInputs() & (1U << 3)) {
                state = 0;
            }
        }
    }
    return state;
}

//----------------------------------------------
void GetGPSString(const char *const match, int wait, char *out, int buflength) {
    char ck;
    int cnt = 0;
    out[0] = 0;
    if (debugflag)
        syslog(LOG, "GetGPSString %s", match);

    int port_gps = OpenSerialPort(PortNameGPS, B4800);
    if (port_gps == 0) {
        syslog(LOG, "failed to open %s", PortNameGPS);
        return;
    }

    while (CheckSerial(port_gps, wait)) {
        ck = '.';
        if (ReadSerial(port_gps, &ck, 1) == 1) {
            if (ck == match[cnt]) {
                cnt++;
                if (match[cnt] == 0) break;
            } else cnt = 0;
        }
    }

    if (cnt == 0) {
        close(port_gps);
        syslog(LOG, "no match from GPS");
        return;
    }
    cnt = 0;
    while (CheckSerial(port_gps, 128)) {
        ReadSerial(port_gps, &ck, 1);
        if (ck == '*' || ck == 0x0d || ck == 0x0a) break;
        out[cnt] = ck;
        cnt++;
        if (cnt == buflength) break;
    }
    out[cnt] = 0;

    close(port_gps);
}

//----------------------------------------------
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

    if (!gpstxt[0]) {
        return (0);
    }

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

//----------------------------------------------
long FileSize(const char *const name) {
    int f;
    long a;
    f = open(name, O_RDONLY);
    if (f == -1) return (-1);
    a = lseek(f, 0, SEEK_END);
    close(f);
    return (a);
}

//----------------------------------------------
void TouchFile(char *name) {
    FILE *f;
    if (FileSize(name) < 1) return;
    f = fopen(name, "r+b");
    if (f < (FILE *) 1) f = fopen(name, "w+b");
    if (f > (FILE *) 1) fclose(f);
}

//----------------------------------------------
int GetGPS() {
    long dd;
    int ret;
    struct tm t;
    gpsok = 0;
    ret = 0;

    u8 timeWasSetBefore = timeisset;

    ret = GetGPSnew();

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

    yymmdd_date = (gpsdate % 100);
    yymmdd_date = yymmdd_date * 100 + ((gpsdate / 100) % 100);
    yymmdd_date = yymmdd_date * 100 + ((gpsdate / 10000) % 100);

    dd = gpstime / 100;
    t.tm_sec = (dd % 100);
    t.tm_min = ((dd / 100) % 100);
    t.tm_hour = ((dd / 10000) % 100);
/*
    time_t tt=mktime(&t);
    if(tt!=0)
      stime(&tt);
*/
    sprintf(hhmmss_time, "%02d%02d%02d", t.tm_hour, t.tm_min, t.tm_sec);
    sprintf(txt, "date -s '%04d-%02d-%02d %02d:%02d:%02d'",
            t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
            t.tm_hour, t.tm_min, t.tm_sec);

    system(txt);
    syslog(LOG,"%s", txt);

    timeisset = 1;
    TouchFile(uploadname);

    if (sun_getgpsfromfile) {
        FILE *f = fopen("gps.txt", "r");
        if (f > (FILE *) 0) {
            fscanf(f, "%lf %lf\n", &gpslat, &gpslon);
            fclose(f);
        }
    }
    if (0 == timeWasSetBefore) {
        system("hwclock -w");
        syslog(LOG, "Doing hwclock -w");
    }
    return (1);
}

//----------------------------------------------
void Reboot() {
    syslog(LOG, "Rebooting");
    system("reboot");
}

//----------------------------------------------
void UpdateEthernet() {
    char txt[2048];
    for (int i = 0; i < 2; i++) {
        sprintf(txt, "/sys/class/net/eth%d/carrier", i);
        int fd = open(txt, O_RDONLY | O_NOCTTY | O_CLOEXEC);
        if (fd != -1) {
            char c = '.';
            read(fd, &c, 1);
            close(fd);
            if (c == '1') {
                fd = open("/proc/net/route", O_RDONLY | O_NOCTTY | O_CLOEXEC);
                if (fd != -1) {
                    int s = read(fd, txt, sizeof(txt) - 1);
                    close(fd);
                    if (s > 0) {
                        txt[s] = 0;
                        char ethx[256];
                        sprintf(ethx, "eth%d", i);
                        if (strstr(txt, ethx) == 0) {
                            sprintf(txt, "ifdown eth%d ; ifup eth%d", i, i);
                            syslog(LOG,"%s", txt);
                            system(txt);
                        }
                    }
                }
            }
        }
    }
}

//----------------------------------------------
int KbdCheck() {
    UpdateEthernet();

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
                remove(cmdname);
                Reboot();
            }
            if (strncmp(txt, "exit", 4) == 0) {
                fclose(f);
                remove(cmdname);
                CleanUp(0);
            }
            if (strncmp(txt, "exec", 4) == 0) {
                char name[32];
                sscanf(txt + 4, "%s", name);
                StatusWriter(EXECUTE);
                syslog(LOG, "%s\n", name);
                system(name);
                goto donekbd;
            }
            if (strncmp(txt, "del", 3) == 0) {
                char name[32];
                sscanf(txt + 3, "%s", name);
                StatusWriter(DELETE);
                syslog(LOG, "%s\n", name);
                remove(name);
                goto donekbd;
            }
            if (strncmp(txt, "paused", 5) == 0) {
                paused = 1;
                StatusWriter(paused);
                goto donekbd;
            }
            if (strncmp(txt, "resume", 6) == 0) {
                paused = 0;
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
            remove(cmdname);
        }
    }

    return (0);
}

//----------------------------------------------
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
            if ((dir->d_type != DT_DIR) && (strstr(dir->d_name, ".pak"))) {
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
        system(cmd);
        rmdir(oldestname);
        return (1);
    }
    sprintf(oldestname, "u%03x.pak", minu);
    syslog(LOG, "Deleting %s to free up space\n", oldestname);
    remove(oldestname);
    return (1);
}

//----------------------------------------------
int VerboseDeleteOldest() {
    // if verbose file creation is the method used it will use this function to delete the oldest file folder structure
    struct dirent *dir;
    DIR *d;

    // we need to look for the min to find the oldest of the file and directories
    // folders are saved as date_time and files as serial_date_time
    int min_directory_date, min_directory_time, min_file_date, min_file_time, t_date, t_time;
    char oldestname[40];
    char cmd[20];
    char serial[256];

    msleep(128);
    syslog(LOG, "DeleteOldest");
    DelOldestCnt++;
    min_directory_date = min_directory_time = min_file_date = min_file_time = 999999;

    d = opendir(".");
    if (d != NULL) {
        while (dir = readdir(d)) {

            if ((dir->d_name[6] == '_') && dir->d_type == DT_DIR) {
                sscanf(&dir->d_name[0], "%u_%u", &t_date, &t_time);
                if (min_directory_date > t_date || min_directory_time > t_time) {
                    min_directory_date = t_date;
                    min_directory_time = t_time;
                }
            }
            if ((dir->d_type != DT_DIR) && (strstr(dir->d_name, ".pak")) && (strstr(dir->d_name, "upload") == NULL)) {
                sscanf(&dir->d_name[strlen(dir->d_name) - 17], "%u_%u", &t_date, &t_time);
                if (min_file_date > t_date || min_file_time > t_time) {
                    // save the serial, this is just in case the file name serial is different after a change in spectrometer

                    strncpy(serial, dir->d_name, strlen(dir->d_name) - 18);
                    // std::string temp_str = dir->d_name;                    
                    // serial = temp_str.substr(0, strlen(dir->d_name) - 18);

                    min_file_date = t_date;
                    min_file_time = t_time;
                }
            }

        }
        closedir(d);
    }

    // We don't need to check time here because if either the file or directory date are at max value the year is not valid anyway
    if ((min_directory_date == 999999) && (min_file_date == 999999)) {
        if (DelOldestCnt > 200)
            Reboot();
        return (0);
    }
    if (min_directory_date != 999999) {
        sprintf(oldestname, "%06d_%06d", min_directory_date, min_directory_time);
        sprintf(cmd, "rm -f %s/*", oldestname);
        system(cmd);
        rmdir(oldestname);
        return (1);
    }
    sprintf(oldestname, "%s_%06d_%06d.pak", serial, min_file_date, min_file_time);
    syslog(LOG, "Deleting %s to free up space\n", oldestname);
    remove(oldestname);
    return (1);
}

//----------------------------------------------
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

    if (strlen(txt) > 2) {
        syslog(LOG, "Successfully read spectrometer serial number '%s'\n", txt + 1);
        if (instrumentname[0] != 0) {
            syslog(LOG, "Will not chane name. Instrument name already set to '%s' in config file\n", instrumentname);
        } else {
            strncpy(instrumentname, txt + 1, 15);
        }
    }
}


//----------------------------------------------
void ReadTemperature() {
    int j;
    short tmp;
    FlushSerial(port_spec, 128);
    txt[0] = '?';
    txt[1] = 't';
    WriteSerial(&txt, 2);

    j = 0;
    txt[0] = 0;
    while (CheckSerial(port_spec, 512) && (j < 2)) {
        j += ReadSerial(port_spec, &tmp, 2 - j);
    }
    if (j == 2) {
        temperature = tmp;
        temperature /= 100.0;
        temperature -= 273.15;
        syslog(LOG, "Temperature inside spectrometer: %.2f deg C\n", temperature);
    }
}

//----------------------------------------------
#include "algofornovac.c"

    char cfgfile[16];
    int cfgonce = 0;
    double lastAngle = 360.0;

    void CheckSunAngle() {
        if ((gpslat == 0.0) && (gpslon == 0.0)) {
            return;
        }

        double angle = GetSunAngle();
        syslog(LOG, "SunAngle=%lf lastAngle=%lf minElevAngle=%lf stratoangle=%d\n", angle, lastAngle, minElevAngle,
               stratoAngle);

        if (lastAngle < 360.0) {
            if ((angle < minElevAngle) && (lastAngle >= minElevAngle)) {
                syslog(LOG, "Pausing due to elevation angle\n");
                paused = 1;
                SwitchRelay(0);
            } else {
                if ((angle > minElevAngle) && (lastAngle <= minElevAngle)) {
                    syslog(LOG, "Resume due to elevation angle\n");
                    paused = 0;
                    SwitchRelay(1);
                }
            }
        }
        lastAngle = angle;

        if (paused == 0) {
            if (FileSize(cfgStratoName) > 0) {
                if (debugflag)
                    syslog(LOG, "Stratosphere cfg found %s\n", cfgStratoName);

                if (angle < stratoAngle) {
                    strcpy(cfgfile, cfgStratoName);
                    ReadSettingFile(cfgfile);
                } else {
                    strcpy(cfgfile, cfgtxtname);
                }
            }
        }
    }

    //----------------------------------------------
    int cfgMainFileNumber = 0;

    void HandleCfgMain() {
        if (FileSize(cfgMainName) > 0) {
            if (debugflag)
                syslog(LOG, "%s found\n", cfgMainName);
        } else {
            return;
        }

        FILE *f = fopen(cfgMainName, "r");
        if (f < (FILE *) 1) {
            return;
        }

        char txt[256];
        char fileName[256];
        int lineNr = 0;
        while (fgets(txt, sizeof(txt) - 1, f)) {
            sscanf(txt, "%s", fileName);
            if (FileSize(fileName) > 0) {
                if (lineNr == cfgMainFileNumber) {
                    strcpy(cfgfile, fileName);
                    ReadSettingFile(cfgfile);
                    if (debugflag)
                        syslog(LOG, "Selecting file %s (%d) from %s", cfgfile, lineNr, cfgMainName);

                }
                lineNr++;
            }
        }
        fclose(f);
        cfgMainFileNumber++;
        if (cfgMainFileNumber >= lineNr) {
            cfgMainFileNumber = 0;
        }
    }


    //----------------------------------------------
    // the file cfgonce.txt contains measurements that should only be
    // executed one time

    void HandleCfgOnce() {
        if (cfgonce) {
            cfgonce = 0;
            remove(cfgoncename);
            strcpy(cfgfile, cfgtxtname);
            ReadSettingFile(cfgfile);
        } else {
            if (FileSize(cfgoncename) > 0) {
                cfgonce = 1;
                strcpy(cfgfile, cfgoncename);
                ReadSettingFile(cfgfile);
            }
        }
    }

    //----------------------------------------------
    int ftpiter;

    void CheckFtpIteration() {
        if (serverip) {
            if (ftpiter <= 0) {
                if (FileSize(uploadname) > 0) {
                    if (ftpconnect() == 0) {
                        if (debugflag > 0) syslog(LOG, "Could not send file");
                    } else {
                        remove(uploadname);
                    }
                }
                ftpiter = ftpiterscans;
            } else
                ftpiter--;
        }
    }

    //----------------------------------------------
    void CheckRenamePakFiles() {
        long siz_ul, siz_work;

        siz_work = MemoryFileSize();
        if (siz_work <= 0) return;

        siz_ul = FileSize(uploadname);
        if (siz_ul == 0) remove(uploadname);

        if (siz_ul <= 0) {
            SaveMemoryFile(uploadname);
        } else {
            if (realtime != 2) {
                if (siz_ul > 0) {
                    if (verboseFileName == 1) {
                        // set date format from ddmmyy to yymmdd
                        // std::string str_date = std::to_string(yymmdd_date).substr(0, 6);
                        // std::string str_time = hhmmssSS_time;
                        sprintf(txt, "%s_%s_%s.pak", instrumentname, yymmdd_date, hhmmss_time);
                        
                        SaveMemoryFile(txt);
                        uploadcnt++;
                    } else {
                        // old method of creating generic file names with a count
                        sprintf(txt, "u%03x.pak", uploadcnt);
                        SaveMemoryFile(txt);
                        uploadcnt++;
                    }
                }
            }
        }
        if (uploadcnt >= 512) Reboot();
    }

    //----------------------------------------------
    void RenamePakFile(const char *const name) {
        int idx;

        if (FileSize(name) <= 0) return;
        if (debugflag > 0) syslog(LOG, "Found existing %s\n", name);
        for (idx = 0; idx < 100; idx++) {
            sprintf(txt, "upload%d.pak", idx);
            if (FileSize(txt) <= 0) {
                sprintf(txt, "cp %s upload%d.pak", name, idx);
                system(txt);
                remove(name);
                return;
            }
            if (debugflag > 1) syslog(LOG, "%s exists\n", txt);
        }
    }


    //----------------------------------------------
    void CleanUp(int retV) {
        SwitchRelay(0);
        printf("E1\n");
        if (smem1) free(smem1);
        printf("E2\n");
        if (sbuf) free(sbuf);
        printf("E3\n");
        if (port_spec) close(port_spec);
        printf("E4\n");
        StatusWriter(STOPPROGRAM);
        printf("E5\n");
        exit(retV);
    }

} // end of namespace

#include "kongo_for_avantes.cxx"

//----------------------------------------------
int main(int argc, char *argv[]) {

    if (debugflag > 0) syslog(LOG, "Compiled " __DATE__  " " __TIME__ "\n");

    // Load the kernel-modules required for the GPS
    system("modprobe usbserial");
    system("insmod ../pl2303.ko");

    gpsalt = gpslat = gpslon = 0;
    ftpiterscans = 10;

    ftpiter = 0;
    smem1 = (long *) 0;
    sbuf = (u16 *) 0;
    hostipaddr = 0;
    memset(instrumentname, 0, 16);
    memset(spectrometerType, 0, 10);

    StatusWriter(STARTPROGRAM);

    smem1 = (long *) dosallocate(4 * (maxlen));
    sbuf = (u16 *) dosallocate(buffertbytes);
    if ((!smem1) || (!sbuf)) {
        StatusWriter(MEMERR);
        CleanUp(0);
    }

    ReadSettingFile(cfgtxtname);
    GetGPS();

    if (realtime != 2) {
        MovePakFiles::MovePakFiles();
    } else {
        RenamePakFile(uploadname);
    }

    InitSerial();

    SwitchRelay(0);
    sleep(1);
    KbdCheck();

    SwitchRelay(1);
    FlushSerial(port_spec, 128);
    StatusWriter(WAIT4S);

    if (isAvantesSpectrometer()) {
        ResetSpectrometer();
    } else {
        if (isHR2000Spectrometer() || isUSB2000PlusSpectrometer()) {
            ChangeBaudrate();
        } else {
            CheckSerial(port_spec, 4000);
        }
        ResetSpectrometer();
        ReadSerialNumber();
    }
    CheckBattery();

    SendMotor(0);

    if (skipmotor != 1) {
        short dir, node;
        if (debugflag > 0) StatusWriter(HOMEMOTOR);
        for (node = 0; node < 2; node++) {
            if (stepsperround[node]) {
                dir = 1;
                if (stepsperround[node] < 0) dir = -1;
                HomeMotor(node, dir);
            }
        }
    }
    CheckBattery();

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
            }
            if (measpt == 0) {
                // when arrived here, we will just start the MEAS= list from the beginning

                if (isHR4000Spectrometer() || isUSB4000Spectrometer()) {
                    ReadTemperature();
                }
                HandleCfgMain();
                CheckSunAngle();
                HandleCfgOnce();
                CheckRenamePakFiles();
                CheckFtpIteration();
            }
            if (paused || (measurecnt == 0))
                msleep(1000);
            else {
                // when arrived here, we are not pausing
                if (meas[measpt].intsum == 0 && meas[measpt].extsum == 0) {
                    if (meas[measpt].pos) HomeMotor(0, meas[measpt].pos);
                    if (meas[measpt].pos2) HomeMotor(1, meas[measpt].pos2);
                } else {
                    if (debugflag) syslog(LOG, "Moving to %s (%d)\n", meas[measpt].name, measpt);
                    lastpos[0] = motorposition[0];
                    lastpos[1] = motorposition[1];
                    MoveMotor(meas[measpt].pos, meas[measpt].pos2);
                    if (debugflag) syslog(LOG, "Measuring %s (%d)\n", meas[measpt].name, measpt);

                    initspectrometer(meas[measpt].chn, meas[measpt].intsum);

                    SetTaskPrio(TASKPRIO_HIGH);
                    // raise priority so we can make fast wind measurements
                    for (repeat = 0; repeat < meas[measpt].repetitions && !KbdCheck();) {
                        if ((meas[measpt].flag & 0x08) && repeat) {
                            if (repeat & 1) MoveMotor(lastpos[0], lastpos[1]);
                            else MoveMotor(meas[measpt].pos, meas[measpt].pos2);
                        }
                        if (doscan()) {
                            // something went wrong, reinitialize and try again
                            SetTaskPrio(TASKPRIO_LOW);
                            // lower priority to allow other tasks to get time
                            ResetSpectrometer();
                            initspectrometer(meas[measpt].chn, meas[measpt].intsum);
                            SetTaskPrio(TASKPRIO_HIGH);
                            // raise priority so we can make fast wind measurements
                        } else {
                            ++repeat;
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

