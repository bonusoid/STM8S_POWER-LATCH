//writer : bonusoid (bonus adityas)
//15 january 2022

#ifndef __POWERMAN_H
#define __POWERMAN_H

//POWER-LATCH Port	
#define POW_LATCH	P5	//0 : Power unlatched ; 1 : Power latched
#define POWLATCHODR	PE_ODR
#define POWLATCHIDR	PE_IDR
#define POWLATCHDDR	PE_DDR
#define POWLATCHCR1	PE_CR1
#define POWLATCHCR2	PE_CR2
#define POWLATCH_MASKL	P5_MASKL
#define POWLATCH_MASKH	P5_MASKH

//POWER-DETECT Port
#define POW_DET		P4	//0 : Power-Key unpressed ; 1 : Power-Key pressed
#define POWDETODR	PF_ODR
#define POWDETIDR	PF_IDR
#define POWDETDDR	PF_DDR
#define POWDETCR1	PF_CR1
#define POWDETCR2	PF_CR2
#define POWDET_MASKL	P4_MASKL
#define POWDET_MASKH	P4_MASKH
#define POWKEY_UNPRESSED	0
#define POWKEY_PRESSED		P4_MASKH

//Charging Status & Battery Monitor Port
#define CHG_PR		P6	//0 : Charging in Progress
#define CHG_FL		P7	//0 : Charging Complete
#define BAT_MON		P2	//Set PB2 as ADC Input
#define BAT_LVL		3	//Use ADC1 Channel 3
#define CHGODR		PB_ODR
#define CHGIDR		PB_IDR
#define CHGDDR		PB_DDR
#define CHGCR1		PB_CR1
#define CHGCR2		PB_CR2
#define CHGST_MASKL	0x3F
#define CHGST_MASKH	0xC0

//Charging Status Port Value
#define CHG_PROGRESS	0x80	//CHG_FL=1,CHG_PR=0
#define CHG_FULL	0x40	//CHG_FL=0,CHG_PR=1
#define CHG_NOCHG	0xC0	//CHG_FL=1,CHG_PR=1
#define CHG_NOBAT	0x00	//CHG_FL=0,CHG_PR=0

//Key Response Parameter
#define KDELAY	20	//Anti-Bouncing delay
#define KLONGP	100	//Long-Press count limit

void powerman_init(); 	//GPIOs Initialization

void Power_Latch();	//Activate Power-Latch
void Power_Unlatch();	//Release Power-Latch

unsigned char read_pkey();  	//Check Powerkey State

unsigned int vbat_mon();	//Battery Voltage reading
unsigned char chgst_mon();	//Charging Status reading

#endif
