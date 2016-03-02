#ifndef CURIE_TIME_RTC_H
#define CURIE_TIME_RTC_H

#include "api/RTCAPI.h"

#define RTC_CCVR    (volatile int*)0xb0000400 // Current Counter Value Register
#define RTC_CMR     0xb0000404 // Counter Match Register
#define RTC_CLR     (volatile int*)0xb0000408 // Counter Load Register
#define RTC_CCR     0xb000040C // Counter Control Register
#define RTC_STAT    0xb0000410 // Interrupt Status Register
#define RTC_RSTAT   0xb0000414 // Interrupt Raw Status Register
#define RTC_EOI     0xb0000418 // End of Interrupt Register

class CurieTimeRTC : public RTCAPI
{
public:
    CurieTimeRTC();

    virtual unsigned long getTime();
    virtual void setTime(unsigned long t);
};

#endif // CURIE_TIME_RTC_H
