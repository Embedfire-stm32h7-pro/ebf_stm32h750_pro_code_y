#ifndef __RTC_H__
#define __RTC_H__

#include "stm32h7xx.h"

extern RTC_HandleTypeDef Rtc_Handle;
// ʱ��Դ�궨��
#define RTC_CLOCK_SOURCE_LSE      
//#define RTC_CLOCK_SOURCE_LSI  

// �첽��Ƶ����
#define ASYNCHPREDIV         0X7F
// ͬ����Ƶ����
#define SYNCHPREDIV          0XFF

// ʱ��궨��
#define RTC_H12_AMorPM			  RTC_HOURFORMAT12_AM  
#define HOURS                     1          // 0~23
#define MINUTES                   1          // 0~59
#define SECONDS                   1          // 0~59


// ���ں궨��
#define WEEKDAY                   1         // 1~7
#define DATE                      1         // 1~31
#define MONTH                     1         // 1~12
#define YEAR                      1         // 0~99

// ������غ궨��
#define ALARM_HOURS               1          // 0~23
#define ALARM_MINUTES             1          // 0~59
#define ALARM_SECONDS             10          // 0~59

#define ALARM_MASK                RTC_ALARMMASK_DATEWEEKDAY
#define ALARM_DATE_WEEKDAY_SEL    RTC_ALARMDATEWEEKDAYSEL_DATE
#define ALARM_DATE_WEEKDAY        1
#define RTC_Alarm_X               RTC_ALARM_A

// ʱ���ʽ�궨��
#define RTC_Format_BINorBCD  RTC_FORMAT_BIN

// ������Ĵ����궨��
#define RTC_BKP_DRX          RTC_BKP_DR0
// д�뵽���ݼĴ��������ݺ궨��
#define RTC_BKP_DATA         0X32F2 
                                  
void RTC_CLK_Config(void);
void RTC_TimeAndDate_Set(void);
void RTC_TimeAndDate_Show(void);
void RTC_AlarmSet(void);

#endif // __RTC_H__
