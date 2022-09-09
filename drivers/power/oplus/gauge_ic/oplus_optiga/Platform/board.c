//#include "GPIO.h"
#include "board.h"
#include "../oplus_optiga.h"
#include <linux/delay.h>
uint32_t g_ulBaudLow;				//1 * BIF_DEFAULT_TIMEBASE
uint32_t g_ulBaudHigh;				//3 * BIF_DEFAULT_TIMEBASE
uint32_t g_ulBaudStop;				//>5 * BIF_DEFAULT_TIMEBASE
uint32_t g_ulResponseTimeOut;		//>20 * BIF_DEFAULT_TIMEBASE
uint32_t g_ulResponseTimeOutLong;	//>32ms for ECC completion
uint32_t g_ulBaudPowerDownTime;		//>196us
uint32_t g_ulBaudPowerUpTime;		//>10ms
uint32_t g_ulBaudResetTime;			//>1ms

uint32_t g_culNvmTimeout;			//5.1ms

void timing_init(void)
{
	g_ulBaudLow 				= 8;		//1 * tau;
	g_ulBaudHigh 				= 29;		//3 * tau;
	g_ulBaudStop				= 51;		//5 * tau;
	g_ulResponseTimeOut 		= 200;		//10 * tau;
	g_ulResponseTimeOutLong		= 60000;	//32ms;

	g_ulBaudPowerDownTime		= 2000;		//2000 = 1ms
	g_ulBaudPowerUpTime			= 20000;	//20000 = 10 ms
	g_ulBaudResetTime			= 2000; 	//2000 = 1ms

	g_culNvmTimeout 			= 10000;	//5.1ms
}

uint8_t get_pin()
{
	return get_optiga_pin();
}

void set_pin(uint8_t level)
{
	set_optiga_pin(level);
}

void set_pin_dir(uint8_t dir)
{
	set_optiga_pin_dir(dir);
}

void ic_udelay(volatile uint32_t ul_ticks)
{
	if (ul_ticks > 1000){
		mdelay(ul_ticks/1000);
	}else{
		udelay(ul_ticks);
	}
}

