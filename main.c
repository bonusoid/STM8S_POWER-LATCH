
//Power Latch Demo
//Display : LCD Nokia 1202 (2 wire)
//writer : bonusoid (bonus adityas)
//15 january 2022

#include"delay.h"
#include"delay.c"
#include"periph_stm8s.h"
#include"periph_stm8s.c"
#include"lcd_n1202.h"
#include"lcd_n1202.c"
#include"powerman.h" //Power Management Library
#include"powerman.c" //Power Management Library


unsigned char pkeyp,pkeylock;	//Powerkey value, Powerkey lock flag
unsigned char page_id,chg_disp; //Page ID flag, Charging display flag
unsigned char ktime;	//Long-Press counter
unsigned char scf;	//Short-Click toggle flag

unsigned int vbat;	//Battery Voltage value
unsigned char chgst;	//Charging Status

void loop();	//Main Loop

void Page_Charging();	//Charging Mode/Page display function
void Page_Main();	//Main (Power-On) Mode/Page display function

unsigned char read_pkey();	//Read Powerkey state
void update_pkey();		//Execute suitable action for Powerkey state

void on_single_click();		//Action/funtion for Single-Click on Power-On Mode
void chg_single_click();	//Action/funtion for Single-Click on Charging Mode

void disp_bat_status();	//Display Charging Status & Battery Voltage


//^^^^^^^^^^ INIT ^^^^^^^^^^//
int main()
{
  //Peripherals Initialization
  clock_init();
  delay_init();
  powerman_init();
  adc_init();
  lcdn1202_init();
  LCD_clear();

  //Initial values
  pkeylock = 1; //lock powerkey for next press/click
  chg_disp = 0; //Charging Status & Battery Voltage is not displayed

  //First check of POWER-DETECT after ON 
  pkeyp = read_pkey(); //check if Powerkey is pressed
  if(pkeyp==POWKEY_PRESSED) Page_Main(); //if pressed then Power-On Mode, go to Main Page
  else Page_Charging(); //if not pressed then Charging Mode, go to Charging Page

  loop();
  return 0;
}
//__________ INIT __________//


//^^^^^^^^^^ MAIN LOOP ^^^^^^^^^^//
void loop()
{
	
	ktime = 0; //start Long-Press counter with 0

	while(1)
	{
		if((read_pkey()==0)&&(pkeylock!=0)) pkeylock = 0; //unlock Powerkey from Long-Press
		else;

		while((read_pkey()!=POWKEY_UNPRESSED)&&(pkeylock==0)) //check if Powerkey is pressed and unlocked
		     {
			pkeyp = read_pkey();
			delay_ms(KDELAY); //Anti-Bouncing delay
			ktime++; //increment Long-Press counter
			if(ktime>KLONGP) break; //detect Long-Press
		     }
		update_pkey(); //Execute Action for Powerkey

		disp_bat_status(); //Display Battery Status

	} 	
}
//__________ MAIN LOOP __________//



//^^^^^^^^^^ MAIN PAGES ^^^^^^^^^^//
void Page_Charging() //Charging Mode
{
	page_id = 0; //Charging Page ID
	LCD_clear();

  	chg_disp = 1; //Charging Status displayed
	LCD_drawtext("CHARGING",2,16);
	delay_ms(2000); //disable Powerkey for a while
	pkeyp = 0; //reset Powerkey flag
	scf = 1; //default toggle state of Single Click in Charging Mode
}

void Page_Main() //Power-On Mode
{
	page_id = 1; //Power-On Page ID
	Power_Latch(); //activate Power-Latch
	LCD_clear();

	chg_disp = 0; //Charging Status not displayed
  	LCD_drawtext("POWER ON",2,16);
  	delay_ms(2000); //disable Powerkey for a while
	LCD_clearblock(2,16,79);
	pkeyp = 0; //reset Powerkey flag
	scf = 1; //default toggle state of Single Click in Power-On Mode
}
//__________ MAIN PAGES __________//


//^^^^^^^^^^ POWER LATCH ^^^^^^^^^^//
void update_pkey() //Action options for Powerkey
{
	if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime>KLONGP)) //Long-Press in Power-On Mode
	{
		pkeyp = 0; //reset Powerkey value
		pkeylock = 1; //lock Powerkey for next press/click
		Power_Unlatch(); //release Power-Latch
		LCD_clear();
		LCD_drawtext("POWR OFF",2,16);
		delay_ms(2000); //make sure Page_Charging is not pop-up after Power-Off
		Page_Charging(); //change to Charging Mode
	}
	else if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime<=KLONGP)) //Single-Click in Power-On Mode
	{
		pkeyp = 0; //reset Powerkey value
		on_single_click();
	}

	else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime>KLONGP)) //Long-Press in Charging Mode
	{
		pkeyp = 0; //reset Powerkey value
		pkeylock = 1; //lock Powerkey for next press/click
		Page_Main(); //change to Power-On Mode
	}
	else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime<=KLONGP)) //Single-Click in Charging Mode
	{
		pkeyp = 0; //reset Powerkey value
		chg_single_click();
	}
	else;
	ktime = 0;
}
//__________ POWER LATCH __________//


//^^^^^^^^^^ ADDITIONAL ACTION ^^^^^^^^^^//
void on_single_click() //Single-Click action on Power-On Mode
{
	if(scf==0)
	{
		LCD_BL_ON(); //Backlight On
		scf = 1;
	}
	else if(scf==1)
	{
		LCD_BL_OFF(); //Backlight Off
		scf = 0;
	}
	else{}	
}

void chg_single_click() //Single-Click action on Charging Mode
{
	if(scf==0)
	{
		LCD_drawtext("        ",4,16); //Clear text
		scf = 1;
	}
	else if(scf==1)
	{
		LCD_drawtext("bonusoid",4,16); //Display text
		scf = 0;
	}
	else{}
}
//__________ ADDITIONAL ACTION __________//


//^^^^^^^^^^ BATTERY STATUS ^^^^^^^^^^//
void disp_bat_status()
{
	//Check Charging Status
	chgst = chgst_mon();

	if(chgst==CHG_PROGRESS) LCD_drawtext("BCHRG",0,0);
	else if(chgst==CHG_FULL) LCD_drawtext("BFULL",0,0);
	else if(chgst==CHG_NOCHG) LCD_drawtext("NOCHG",0,0);
	else if(chgst==CHG_NOBAT) LCD_drawtext("NOBAT",0,0);
	else LCD_drawtext("UNKWN",0,0);

	//Check Battery Voltage
	LCD_drawtext("mV",0,80);
	vbat = vbat_mon();
	LCD_drawint(vbat,0,48);
}
//__________ BATTERY STATUS __________//
