//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: BUTTONS.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __BUTTONS_H__
#define __BUTTONS_H__

 

//---------------------GLOBAL DEFINITIONS--------------------------
#define BUSV_COUNT 4
 
#define KEY_CMONSTER    0x08
#define KEY_BUSVCMONSTER 0x04  

//---------------------GLOBAL VARIABLES--------------------------
extern short int cMonsterPressed; 
 
extern uint16_t timerBusVNoChange;
extern uint16_t timerBusVChange;

//---------------------GLOBAL PROTOTYPES--------------------------
void ButtonInit(void);
void ButtonSample(void);
uint8_t ButtonChanged(void);

#endif
