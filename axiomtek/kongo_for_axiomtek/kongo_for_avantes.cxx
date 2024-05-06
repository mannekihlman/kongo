#include "as5216.h"

namespace {
    long avantes_DeviceHandle;
    unsigned short avantes_NrPixels;
    MeasConfigType avantes_PrepareMeasData;
    int avantes_timeout;

    double avantes_spectrum[MAX_NR_PIXELS];
    double avantes_inttime = 100.0;
    double avantes_numavg = 1.0;

    //----------------------------------------------
    void PrepareAvantesSpectrometer() {
        int l_Port = AVS_Init(0);
        if (l_Port <= 0) {
            syslog(LOG, "AVS_Init failed");
            return;
        }

        AvsIdentityType l_Active;
        unsigned int l_reqsize;
        if (AVS_GetList(sizeof(AvsIdentityType), &l_reqsize, &l_Active) < 1) {
            syslog(LOG, "AVS_GetList failed");
            return;
        }

        avantes_DeviceHandle = AVS_Activate(&l_Active);
        syslog(LOG, "Successfully read spectrometer serial number '%s'\n", l_Active.SerialNumber);
        if (instrumentname[0] != 0) {
            syslog(LOG, "Will not chane name. Instrument name already set to '%s' in config file\n", instrumentname);
        } else {
            strncpy(instrumentname, l_Active.SerialNumber, 15);
        }

        if (ERR_SUCCESS != AVS_GetNumPixels(avantes_DeviceHandle, &avantes_NrPixels)) {
            syslog(LOG, "AVS_GetNumPixels failed");
        } else {
            syslog(LOG, "Number of pixels %d\n", avantes_NrPixels);
            StatusWriter(CONTACTSPEC);
        }
    }

    //----------------------------------------------
    void callback(AvsHandle *handle, int *result) {
        if (*result == 0) {
            unsigned int l_Time;
            if (ERR_SUCCESS == AVS_GetScopeData(avantes_DeviceHandle, &l_Time, avantes_spectrum)) {
                avantes_timeout = 0;
            }
        }
    }

    //----------------------------------------------
    void StartAvantesMeasurement() {
        avantes_PrepareMeasData.m_StartPixel = 0;
        avantes_PrepareMeasData.m_StopPixel = avantes_NrPixels - 1;

        if (startchn < stopchn) {
            avantes_PrepareMeasData.m_StartPixel = startchn;
            avantes_PrepareMeasData.m_StopPixel = stopchn;
        }

        avantes_PrepareMeasData.m_IntegrationTime = avantes_inttime;
        avantes_PrepareMeasData.m_IntegrationDelay = 0;
        avantes_PrepareMeasData.m_NrAverages = avantes_numavg;

        avantes_PrepareMeasData.m_CorDynDark.m_Enable = 0;
        avantes_PrepareMeasData.m_CorDynDark.m_ForgetPercentage = 0;
        avantes_PrepareMeasData.m_Smoothing.m_SmoothPix = 0;
        avantes_PrepareMeasData.m_Smoothing.m_SmoothModel = 0;
        avantes_PrepareMeasData.m_SaturationDetection = 0;
        avantes_PrepareMeasData.m_Trigger.m_Mode = 0;
        avantes_PrepareMeasData.m_Trigger.m_Source = 0;
        avantes_PrepareMeasData.m_Trigger.m_SourceType = 0;
        avantes_PrepareMeasData.m_Control.m_StrobeControl = 0;
        avantes_PrepareMeasData.m_Control.m_LaserDelay = 0;
        avantes_PrepareMeasData.m_Control.m_LaserWidth = 0;
        avantes_PrepareMeasData.m_Control.m_LaserWaveLength = 0;
        avantes_PrepareMeasData.m_Control.m_StoreToRam = 0;

        int l_Res = AVS_PrepareMeasure(avantes_DeviceHandle, &avantes_PrepareMeasData);
        if (ERR_SUCCESS != l_Res) syslog(LOG, "AVS_PrepareMeasure failed");

        const short l_NrOfScans = 1;

        l_Res = AVS_MeasureCallback(avantes_DeviceHandle, &callback, l_NrOfScans);
        if (ERR_SUCCESS != l_Res) syslog(LOG, "AVS_MeasureCallback failed");
    }

    //----------------------------------------------
    int AvantesAddScan(short isum, short chn, long sleepsum) {
        avantes_timeout = 1;
        StartAvantesMeasurement();

        // save last measurement while waiting so that we can measure faster
        SaveScan();
        if (isum == 0) {
            GetCPUTime();
            starttime = gpstime;
            memset(smem1, 0, sizeof(long) * maxlen);
        }

        if (sleepsum < 0) sleepsum = -sleepsum;
        msleep(sleepsum);

        long timeout = 0;
        while (avantes_timeout > 0) {
            msleep(10);
            timeout += 10;
            if (timeout > sleepsum) {
                return 1;      // 1 indicates error
            }
        }
        speclen = 1 + avantes_PrepareMeasData.m_StopPixel - avantes_PrepareMeasData.m_StartPixel;

        if (speclen > maxlen) speclen = maxlen;
        maxv = 0;
        for (int j = 0; j < speclen; j++) {
            double a = avantes_spectrum[j] * avantes_numavg;
            smem1[j] += a;
            if (maxv < a) {
                maxv_idx = j;
                maxv = a;
            }
        }
        if (debugflag > 1) syslog(LOG, "AvantesAddScan successful");
        return 0;
    }

    //----------------------------------------------
    void SetExposureTime(u16 inttime) {
        avantes_inttime = inttime;
        if (debugflag > 1) syslog(LOG, "Exposure time: %d ms\n", inttime);
    }

    //----------------------------------------------
    void SetSumCnt(short sumcnt) {
        avantes_numavg = sumcnt;
        if (debugflag > 1) syslog(LOG, "Internal sum count: %d\n", sumcnt);
    }

    //----------------------------------------------
    float AvantesGetTemperature() {
        float u = 0.0;
        AVS_GetAnalogIn(avantes_DeviceHandle, 0, &u);
        float t = 118.69 - 70.361 * u + 21.02 * u * u - 3.6443 * u * u * u + 0.1993 * u * u * u * u;
        if (debugflag > 0) syslog(LOG, "Avantes temperature: %f\n", t);
        return t;
    }

    //----------------------------------------------
    int AvantesInitspectrometer(short chn, short sumcnt) {
        long m, digitalnoise;

        if (debugflag > 0)
            StatusWriter(INITSPEC);

        long inttime = meas[measpt].inttime;
        meas[measpt].realexptime = inttime;
        if (inttime == 0) {
            inttime = meas[0].realexptime;
            if (inttime < 0) inttime = -inttime;
            if (inttime == 1) inttime = maxIntTime;
            meas[measpt].realexptime = -inttime;
            if (debugflag) syslog(LOG, "Using zenith exposure time: %ld ms\n", inttime);
        } else if (inttime < 0) {
            SetSumCnt(1);
            const short stest = 24;
            short ltest = 300; //no longer a constant because it can change with dynamic handling of exposure
            const long maxcounts = 16384;

            SetExposureTime(stest);
            if (AvantesAddScan(0, chn, stest)) {
                inttime = 20;
            } else {
                if (pchannel != -1) {
                    maxv = AvgChannels(smem1, pchannel, 10, maxlen);
                }

                float digitalNoisePercentage = maxv / maxcounts;
                digitalnoise = maxv;
                if (debugflag > 0)
                    syslog(LOG, "Maxvalue: %d (idx=%d) (inttime=%.0lf)\n", digitalnoise, maxv_idx, avantes_inttime);

                // if the digital noise is already 70% or higher just use the min test
                if (digitalNoisePercentage > m_percent) {
                    inttime = stest;
                } else {
                    SetExposureTime(ltest);
                    AvantesAddScan(0, chn, ltest);
                    if (pchannel != -1) {
                        maxv = AvgChannels(smem1, pchannel, 10, maxlen);
                    }

                    // find percentage of the maxv returned versus full exposure
                    float percFullExposure = maxv / maxcounts;
                    // low, high, and currentTest are for a binary search
                    short low = stest;
                    short high = ltest;
                    short currentTest = ltest;

                    // If we didn't meet exposure limits we need to keep searching to find it
                    // We want exposure to land between 30 and 90 percent
                    while (percFullExposure < .30 || percFullExposure > .90 || currentTest >= maxIntTime) {

                        // create a new exposure time depending on if we need to go up in time or down
                        if (percFullExposure < .30) {
                            // if our exposure is too low on the high side we will double it,
                            // assuming it comes in measuring anywhere from 15% to 45% exposure, doubling should hit within our threshold
                            if (high == currentTest) {
                                low = high;
                                high = high * 2;
                                currentTest = high;
                            } else {
                                // if it comes in too low on a median test we are going to run binary search
                                low = currentTest;
                                currentTest = (int) (high + low) / 2;
                            }
                        } else if (percFullExposure > .90) {
                            // if we come in too high on exposure on a median or high test we will binary search
                            high = currentTest;
                            currentTest = (int) (high + low) / 2;
                        }

                        // kill the binary search if the low and the high ends of exposure come into contact, threshold contact can be changed as needed from 5 ms
                        if ((high - low) < 5) {
                            break;
                        }

                        SetExposureTime(currentTest);
                        AvantesAddScan(0, chn, currentTest);
                        if (pchannel != -1) {
                            maxv = AvgChannels(smem1, pchannel, 10, maxlen);
                        }

                        percFullExposure = maxv / maxcounts;

                    }

                    // Need to reset the ltest in the case that dynamic exposure found a different value
                    ltest = currentTest;
                    m = maxv;

                    if (debugflag > 0) syslog(LOG, "Maxvalue: %d (idx=%d) (inttime=%.0lf)\n", m, maxv_idx, avantes_inttime);

                    // with the work above I no longer see this as useful and I don't want it setting max time if digital noise and m are both overexposed, could only make things worse
                    // TODO depricate and remove
                    // if (m < 1 || digitalnoise == m) {
                    //   inttime = maxIntTime;
                    //   if (debugflag)
                    //     syslog(LOG, "Calculated exposure time: %ld ms\n", inttime);
                    // } else {
                    float a = maxcounts - digitalnoise;
                    float b = ltest - stest;
                    float c = m - digitalnoise;
                    float d = m_percent * a * b / c;
                    inttime = d;
                    inttime += stest;

                    if (debugflag) syslog(LOG, "Calculated exposure time: %ld ms", inttime);
                    if (inttime > maxIntTime) {
                        inttime = maxIntTime;
                        if (debugflag) syslog(LOG, " but forced to %ld ms\n", inttime);
                    } else if (inttime < minIntTime) { // this should no longer be needed with binary search above but leaving it in for posterity
                        if (m < 200) inttime = maxIntTime;
                        else inttime = minIntTime;
                        if (debugflag) syslog(LOG, " but forced to %ld ms\n", inttime);
                    }
                }
                // } from if else above that was deprecated TODO remove
                meas[measpt].realexptime = -inttime;
            }
        }
        SetSumCnt(sumcnt);
        SetExposureTime(inttime);

        if (debugflag > 1) StatusWriter(DONEINITSPEC);

        return (0);
    }

}