//writer : bonusoid (bonus adityas)
//15 january 2022

#include"powerman.h"

void powerman_init() //GPIOs Initialization 
{
	POWLATCHDDR |= (OUTPUT<<POW_LATCH);
	POWLATCHCR1 |= (pushpull<<POW_LATCH);
	POWLATCHCR2 |= (speed_2MHz<<POW_LATCH);

	POWDETDDR |= (INPUT<<POW_DET);
	POWDETCR1 |= (floating<<POW_DET);
	POWDETCR2 |= (exti_disabled<<POW_DET);

	CHGDDR |= (INPUT<<CHG_PR) | (INPUT<<CHG_FL);
	CHGCR1 |= (pullup<<CHG_PR) | (pullup<<CHG_FL);
	CHGCR2 |= (exti_disabled<<CHG_PR) | (exti_disabled<<CHG_FL);
}

void Power_Latch() //Activate Power-Latch : latch Power-Switch transistor
{
  	POWLATCHODR |= POWLATCH_MASKH; //POWER-LATCH = 1
}

void Power_Unlatch() //Release Power-Latch : unlatch Power-Switch transistor
{
  	POWLATCHODR &= POWLATCH_MASKL; //POWER-LATCH = 0
}

unsigned char read_pkey() //Check Powerkey State
{
	unsigned char pkeyval;
	
	pkeyval = POWDETIDR & POWDET_MASKH; //Read POWER-DETECT state

	return pkeyval;
}

unsigned int vbat_mon() //Battery Voltage reading
{
	unsigned int vbat;

	//VBAT Calculation Formula (for Single Cell, Ratio -> VADC:VBAT=2:3)

	//vbat = read_adc(BAT_LVL)*64/40*3; //VADC_MAX=3250mV,VBAT_MAX=4875mV
	vbat = read_adc(BAT_LVL)*66/40*3; //VADC_MAX=3333mV,VBAT_MAX=5000mV

	return vbat;
}

unsigned char chgst_mon() //Charging Status reading

{
	unsigned char chgst;

	chgst = CHGIDR & CHGST_MASKH; //Check CHG-PR & CHG-FL

	return chgst;
}
