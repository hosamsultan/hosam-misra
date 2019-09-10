/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "KEYPAD.h"
/*Local Symbols*/
#define KPD_COL_PORT 0x03u
#define KPD_ROW_PORT 0x01u
#define KPD_COL_MASK 0x70u
#define KPD_ROW_MASK 0x0fu
#define KPD_COL_PIN_NUM 4u
#define KPD_ROW_PIN_NUM 0u
/**************************************************/
/*#define KPD_ROW_INIT() DIO_vidWritePortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xff); \
                       DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,0x00)
*/

/*#define KPD_ROW_WRITE(DATA) DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATA) << KPD_ROW_PIN_NUM))
 */

u8 DIO_vidReadPortData(u8 PortName,u8 PortMAsk,u8* VALPTR);
void DIO_vidWritePortDirection(u8 PortName,u8 PortDirection,u8 PortMask);
void DIO_vidWritePortData(u8 PortName,u8 PortData,u8 PortMask);
void DIO_WritePortData(u8 PortName,u8 PortDirection,u8 PortMask);

void KPD_Init(void)
{
	KPD_COL_INIT();
	KPD_ROW_INIT();
	
}
void KPD_ReadVal(unsigned char* ValuePtr)
{
    static const unsigned char KeysLut[]= {'1' , '2' , '3' , '4' , '5' , '6','7' , '8' , '9','*' , '0' , '#'};
	u8 Rowdata=0u;
	u8 ColData;
	u8 LoopTermnate = 0u;
	for(Rowdata=0u ; ((Rowdata < 4u)&&(LoopTermnate == 0u)) ; Rowdata++)
	{
		KPD_ROW_WRITE((u8)(1u<<(u16)Rowdata));
		KPD_COL_READ(&ColData);
		if(ColData != 0u)
		{
			*ValuePtr = KeysLut[(Rowdata*3u) + (ColData/2u)];
			LoopTermnate = 1u;
		}
		else
		{
		    unsigned char c='n';
			*ValuePtr = c;
		}
	}

}

void KPD_COL_INIT(void)
{
    DIO_vidWritePortDirection(KPD_COL_PORT,KPD_COL_MASK,0xffu);
    DIO_WritePortData(KPD_ROW_PORT,KPD_COL_MASK,0x00u);
}

void KPD_ROW_INIT(void)
{
    DIO_vidWritePortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xffu);
    DIO_WritePortData(KPD_ROW_PORT,KPD_ROW_MASK,0x00u);
}

void KPD_COL_READ(u8* VALPTR)
{
    DIO_vidReadPortData(KPD_COL_PORT,KPD_COL_MASK,(VALPTR));
    *(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM;
}

void KPD_ROW_WRITE(u8 DATA)
{
    DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATA)<<KPD_ROW_PIN_NUM));
}
