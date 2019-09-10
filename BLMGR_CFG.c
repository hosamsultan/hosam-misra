/*
 * BLMGR_CFG.c
 *
 * Created: 28/02/2016 06:54:45 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "BLMGR_CFG.h"

#define four 4u
#define five 4u
#define two 4u

BLMGR_DioPinConfig BuzzerConfig       = {PC,1u<<four};
BLMGR_DioPinConfig BlueToothPwrConfig = {PC,1u<<five};
BLMGR_DioPinConfig BluetoothKeyConfig = {PD,1u<<two};
	
