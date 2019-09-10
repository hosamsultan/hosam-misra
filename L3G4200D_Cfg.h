/*
 * L3G4200D_Cfg.h
 *
 * Created: 24/09/2015 09:13:50 م
 *  Author: hossam
 */ 


#ifndef L3G4200D_CFG_H_
#define L3G4200D_CFG_H_
#include "Basic_Types.h"
#include "DIO.h"
#include "SPI.h"
void GYHD_INIT_SLAVE_SELECT(void);
void GYHD_ACTIVATE_SLAVE_SELECT(void);
void GYHD_DEACTIVATE_SLAVE_SELECT(void);

void u8START_TIME_OUT_MS(u16 DELAY_MS,u8* FLAG_PTR);
void _delay_ms(u16 DELAY_MS);



/*SPI Communication Configuration*/



/*Timeout Management Configuration*/
#define u8USE_DELAY 0x00u
#define u8USE_TIMER 0x01u

#define u8TIMEOUT_FUNCTION 0x00u

#if(u8TIMEOUT_FUNCTION == u8USE_DELAY)

#else
#define u8START_TIME_OUT_MS(DELAY_MS,FLAG_PTR)
#endif

/*Self Axis Movement Detection Config*/
#define u8SELF_AXIS_MOV  1u

/*SPI Communication Configuration*/


#endif /* L3G4200D_CFG_H_ */
