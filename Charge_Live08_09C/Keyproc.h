
#ifndef __KEYPRO_H
#define __KEYPRO_H

#define KEY_SCAN_DEBOUNCE_PRESS_COUNT			3 //15 // 3
#define LONG_PRESS_KEY_1S_TIMEOUT			(60)
#define LONG_PRESS_KEY_3S_TIMEOUT			(50) //(65)  //180

#define NO_KEY_WAIT_TIMEOUT				(1000)
typedef struct __Menu_ID_to_Key_Func_Table 
{
 uint8_t Menu_ID;
 uint8_t Key_Value;
 void (*Func_Pointer) (void); //º¯ÊýÖ¸Õë
}Menu_ID_to_Key_Func_Table;

typedef struct
{
 bool     KeyScan_flag;
 uint8_t  State;
 uint8_t  DebounceCounter;
 uint8_t  Key_CurrentValue;
 uint8_t  Key_NewValue;
 uint8_t  Key_Value;
 uint8_t  Key_OldValue;
 uint16_t WaitKeyReleaseTime;
}KeyScanData_t;

typedef enum 
{
	KEY_PAD_IDLE_STATE = 0		,
	KEY_PAD_SCAN_GPIO		    ,
	KEY_PAD_DEBOUNCE_STATE  	,
	KEY_PAD_WAIT_TILL_RELEASE	,
//	KEY_PAD_FINISH				,
}keyscan_states_e;

typedef enum 
{
  	n_NoKey = 0,
	n_KEY_SHORT, 
  	n_KEY_LONGPRESS_1S,     	
  	n_KEY_LONGPRESS_3S,     			
}KeyValue_e;

extern KeyScanData_t  KeyScandData;

void KeyScan_Task(void);
void COR_AP_ShorKey_Common_Handler(void);
#endif