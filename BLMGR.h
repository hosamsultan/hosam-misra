/*
 * BLMGR.h
 *
 * Created: 28/02/2016 04:20:32 م
 *  Author: hossam
 */ 


#ifndef BLMGR_H_
#define BLMGR_H_

#define SLAVE_COMM 0x00u
#define MSTER_COMM 0x01u
#define COMM_CINFIG 0x01u

#define ROLE_CHAIR 0x02u
#define ROLE_CAP  0x03u
#define ROLE_MAPP 0x01u

#define DEVICE_ROLE 0x02u
void BLMGR_Test(void);
void BLMGR_BluetoothInit(void);
void BLMGR_BluetoothStateMachine(void);
void BLMGR_StartDevice(void);
void BLMGR_SetReceiver(u8 Receiver);
void BLMGR_SetDeviceName(u8 const DeviceName[],u16 DeviceNameLength);
void BLMGR_SetBattLevel(u8 BattLevel);

void SECR_GnerateCrc1(u8 TempBuffer8,u8 c, u16* cr ,u32 BLMGR_CrcKey);

#endif /* BLMGR_H_ */
