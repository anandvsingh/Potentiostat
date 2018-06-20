/*This is the main file that controls the Potentiostat
  This file can be used to control various parameters of the device, how to change behaviour of a 
  particular property will be mentioned alongside the variable for better understanding.

*/
//need libm.a
#define TIMER TCC0 //timer to use

#define DAC_CONINTVAL DAC_CONINTVAL_1CLK_gc //DAC settings
#define DAC_REFRESH DAC_REFRESH_16CLK_gc
#define ADC_PRESCALER ADC_PRESCALER_DIV8_gc//ADC sampling speed
#define ADC_OFFSET 0

#define EEPROM_SIZE 1024 //in bytes
#define PROFILES_LENGTH 11 //max number of profiles, limited by memory, we only use 1 for cyclic voltamtery



#define CV_BUFFER_SIZE 16 //must be <=16, memory limit
#define CV_MAX_DATAPOINTS 1500 //DO NOT CHANGE

#define CV 1 //Kept CV as 1 to not cause disagreement with USART operations



//Defining various modes of operation of the system

#define PROFILE_SEL 0
#define PROFILE_OPT 1
#define PROFILE_TEST 2
#define PROFILE_EDIT 3
#define PROFILE_RESULTS 4
#define DISPLAY_RESULTS 5
#define EDIT_NOSEL 0
#define EDIT_SEL 1
#define OPT_START 0
#define OPT_EDIT 1

#define RANGE_10UA 1
#define RANGE_50UA 2
// Numbers are already fixed by the button hardware, named them for ease of code visibility
#define UP 0
#define LEFT 1
#define RIGHT 2
#define DOWN 3
#define INVALID 4

#define bit_get(p,m) ((p) & (m))

#include "dac_driver.h"
#include "adc_driver.h"
#include "lcd.h"
#include "usart_driver.h"
#include <util/delay.h>
#include <stdio.h>
#include <math.h>


#include <string.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "spi_driver.h"
float StDev,mean,HiVal;
int n;


//Defining a profile strucutre (29 bytes), tasks of each profile variable are defined below
typedef struct {
 char name[15];
 uint8_t type;
 int16_t op1; 
 int16_t op2;
 int16_t op3;
 int16_t op4;
 int16_t op5;
 int16_t op6;
 uint8_t curr_range;
} profile;

//profiles are located in both EEPROM and RAM
profile EEMEM profilesEE[PROFILES_LENGTH];
profile profiles[PROFILES_LENGTH];

int buttonHandler(profile profiles[PROFILES_LENGTH], uint8_t* status, uint8_t* profile_index, uint8_t* profile_opt_index, uint8_t* profile_edit_index, uint8_t* profile_edit_sel,int16_t* length);

int16_t CV_test (char* name, int16_t slope, int16_t start, int16_t stop, int16_t scans, int16_t sample_rate, uint8_t curr_range);




USART_data_t USART_data;

void send_string(char* string);
int main()
{
 char Resultstr[12];
 uint8_t status;
 uint8_t profile_index;
 uint8_t profile_opt_index;
 uint8_t profile_edit_index;
 uint8_t profile_edit_sel;

 uint8_t index;
 char temp_string[16];
 uint8_t i;
 int16_t length;


	//4 DIR SWITCH 
	/////////////////////////////////
	//setup pins as input


 PORTA.DIRCLR = PIN4_bm;
 PORTA.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN5_bm;
 PORTA.PIN5CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN6_bm;
 PORTA.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN7_bm;
 PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

	//PORTE.INT0MASK = 0;
	/////////////////////////////////


	//SPST SWITCHES
	/////////////////////////////////
    //setup pins as output
 PORTE.DIRSET = PIN1_bm; //switch0
 PORTE.DIRSET = PIN0_bm; //switch1
 PORTE.DIRSET = PIN2_bm; //switch2
 PORTE.DIRSET = PIN3_bm; //switch3

//set initital switch positions

 PORTE.OUTCLR = PIN1_bm; //switch0
 PORTE.OUTSET = PIN0_bm; //switch1
 PORTE.OUTCLR = PIN2_bm; //switch2
 PORTE.OUTCLR = PIN3_bm; //switch3




//DAC
 DAC_DualChannel_Enable( &DACB,DAC_REFSEL_AVCC_gc,false,DAC_CONINTVAL,DAC_REFRESH);
//1.65V reference
 while (DAC_Channel_DataEmpty(&DACB, CH1) == false) {}
  DAC_Channel_Write(&DACB,2048,CH1);
//intitial
 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,2048,CH0);

	//USART
	/////////////////////////////////
	//TX as output
 PORTC.DIRSET = PIN3_bm;
//RX as input
 PORTC.DIRCLR = PIN2_bm;
 USART_InterruptDriver_Initialize(&USART_data, &USARTC0, USART_DREINTLVL_LO_gc);
// USARTD0, 8 Data bits, No Parity, 1 Stop bit.
 USART_Format_Set(&USARTC0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
//enable interrupts
 USART_RxdInterruptLevel_Set(&USARTC0, USART_RXCINTLVL_LO_gc);
//BUAD RATE to 9600
 USART_Baudrate_Set(&USARTC0, 12 , 0);
 USART_Rx_Enable(&USARTC0);
 USART_Tx_Enable(&USARTC0);




//ADC
	/////////////////////////////////
	// Move stored calibration values to ADC A. 
 ADC_CalibrationValues_Load(&ADCA);
// Set up ADC A to have signed conversion mode and 12 bit resolution.
   ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);
// Set sample rate.
 ADC_Prescaler_Config(&ADCA, ADC_PRESCALER);
	// Set reference voltage on ADC A to be VCC internal
 ADC_Reference_Config(&ADCA, ADC_REFSEL_VCC_gc);
//configure input mode to differential
 ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,ADC_CH_INPUTMODE_DIFF_gc,ADC_DRIVER_CH_GAIN_NONE);
 ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH1,ADC_CH_INPUTMODE_DIFF_gc,ADC_DRIVER_CH_GAIN_NONE);
//configure pins
 ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXNEG_PIN1_gc);
 ADC_Ch_InputMux_Config(&ADCA.CH1, ADC_CH_MUXPOS_PIN2_gc, ADC_CH_MUXNEG_PIN1_gc);
//enable adc
 ADC_Enable(&ADCA);
 lcdInit();
 lcdInit();
 lcdClear();
 lcdHome();
 lcdPrintData("CDAC Kolkata",12);
 _delay_ms(1000);

/////////////////////////////////
	
	//USER INTERFACE
	/////////////////////////////////
 status = PROFILE_SEL;
 profile_index = 1;
 profile_opt_index = OPT_START;
 profile_edit_index = 0;
 profile_edit_sel = EDIT_NOSEL;




//copy profile from EEPROM to SRAM
 for(i = 0; i < PROFILES_LENGTH; i++)
  eeprom_read_block((void*)&(profiles[i]), (const void*)&(profilesEE[i]), sizeof(profile));



//INTERRUPTS
 PMIC.CTRL |= PMIC_LOLVLEX_bm;
 sei();
//END INITIALIZATION

//Creating profile on EEPROM
 strcpy(profiles[1].name,"CV #1          ");
 profiles[1].type = CV;
 profiles[1].op1 = 280; //Slope
 profiles[1].op2 = -700; //Initial value for starting Voltametry
 profiles[1].op3 = 700; //Highest Potential peak that the Voltametry achieves
 profiles[1].op4 = 2; //No of Scans (DO NOT DISTURB UNTIL ABSOLUTELY NECESSARY)
 profiles[1].op5 = 7; //Mv/Samples, this is used to control Data points
 profiles[1].op6 = 0;//This option is disabled for cyclic voltametry however is a part of predefined structure hence left untouched
 profiles[1].curr_range = RANGE_50UA;

 for(i = 0; i < PROFILES_LENGTH; i++) 
  eeprom_write_block((const void*)&(profiles[i]), (void*)&(profilesEE[i]), sizeof(profile));


//MAIN LOOP
 while(1)
 {
   //for selecting profiles
  if(status == PROFILE_SEL) 
  {
   //clear display
   lcdClear();
   lcdHome();
     //lcdGotoXY(Xcord,Ycord) (0,0 at top left cordinates increase downwards and right)
     lcdPrintData("~",1);
     lcdPrintData("Right press to",14);
     lcdGotoXY(0,1);
     lcdPrintData("start CV",8); 
   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }
  //Result display window
  else if(status == DISPLAY_RESULTS)
  {
   //clear LCD
   lcdClear();
   lcdHome(); //Equivalent to lcdGotoXY(0,0)
   lcdPrintData("StDev = ",8);
   StDev = (int) StDev; //Because Sprintf can only handle integers so typecasting is done
   sprintf(Resultstr,"%8d",StDev); //Because lcdPrintData can only handle strings, value is stored in string
   lcdPrintData(Resultstr,8);
   lcdGotoXY(0,1);
   mean = (int) mean;
   lcdPrintData("Mean = ",7);
   sprintf(Resultstr,"%8d",mean);
   lcdPrintData(Resultstr,8);
   lcdGotoXY(0,2);
   HiVal = (int) HiVal;
   lcdPrintData("Max = ",6);
   sprintf(Resultstr,"%8d",HiVal);
   lcdPrintData(Resultstr,8);
   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}

  }
  else if(status == PROFILE_OPT)
  {
  //Shows the start option and waits for instruction
  //clear display
   lcdClear();
   lcdHome();
   lcdPrintData("~",1);
   lcdPrintData("Start test",10);

   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }
  //display while test is going on in the background
  else if(status == PROFILE_TEST)
  {

   lcdClear();
   lcdHome();

   lcdPrintData("Testing....",10);
   length = CV_test(profiles[profile_index].name, profiles[profile_index].op1, profiles[profile_index].op2, profiles[profile_index].op3, profiles[profile_index].op4, profiles[profile_index].op5, profiles[profile_index].curr_range);

   if(length == -1)
   {
    _delay_ms(1000);
    status = PROFILE_EDIT;
   }
   else
    status = PROFILE_RESULTS;
  }
  //display to signify that the test is complete
  else if (status == PROFILE_RESULTS)
  {

   lcdClear();
   lcdHome();
   lcdPrintData("Test Complete",13);
   lcdGotoXY(0,1);
   lcdPrintData("Press right to",14);
   lcdGotoXY(0,2);
   lcdPrintData("view results",12); 


   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }

 }

 return 0;


}
//end of LCD Display and main

//USART interface function (DO NOT DISTURB)
void send_string(char* string)
{
 uint8_t i = 0;
 while(string[i]!='\0')
 {
  do{}
  while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, string[i]);
  i++;
 }
}

int buttonHandler(profile profiles[PROFILES_LENGTH], uint8_t* status, uint8_t* profile_index, uint8_t* profile_opt_index, uint8_t* profile_edit_index, uint8_t* profile_edit_sel, int16_t* length)
{

 uint8_t dir;
 dir = INVALID;
 while(dir == INVALID)
 {
//get action i.e. investigate which direction button is pressed, mapped in the begining
 if(bit_is_set(PORTA.IN,4))
  dir = RIGHT;
 else if(bit_is_set(PORTA.IN,5))
  dir = DOWN;
 else if(bit_is_set(PORTA.IN,6))
  dir = LEFT;
 else if(bit_is_set(PORTA.IN,7))
  dir = UP;
 }



//Changes state of the device based on the present state and button pressed
 if(*status == PROFILE_SEL)
 {

         if(dir == RIGHT)
  {
   *status = PROFILE_OPT;
   *profile_opt_index = 0;
  }
 }
 else if(*status == DISPLAY_RESULTS)
 {
  if (dir==LEFT)
   *status = PROFILE_SEL;
 }
 else if(*status == PROFILE_OPT)
 {

         if(dir == RIGHT)
  {

    *status = PROFILE_TEST;


  }
  else if(dir == LEFT)
  {
   *status = PROFILE_SEL;
  }
 }
 else if(*status == PROFILE_TEST)
 {

 }

 else if (*status == PROFILE_RESULTS)
 {
  if(dir == RIGHT)
  {
   *status = DISPLAY_RESULTS;
  }
  else
   *status = PROFILE_SEL;
 }
 else
  _delay_ms(1000);

 _delay_ms(200);
 return 1;
}

int16_t CV_test (char* name, int16_t slope, int16_t start, int16_t stop, int16_t scans, int16_t sample_rate, uint8_t curr_range)
{



 uint16_t step_time;
 uint16_t steps_per_sample;
 uint16_t steps_taken;
 uint16_t ramps;
 uint16_t samples;
 uint16_t i,j,k;
 int sum = 0,l;
 float st,st2,st3;
 unsigned long variance;
 bool up;

 int16_t current_DAC, min_DAC, max_DAC;

//storing ADC results
 int16_t current[CV_MAX_DATAPOINTS];
 int16_t result_buffer[CV_BUFFER_SIZE];

	//check limits
 if(start<-1600 || start>1600 || stop<-1600 || stop>1600 || slope>9000 || slope<10 || sample_rate<1 || sample_rate>1600)
 {
  lcdClear();
  lcdHome();
  lcdPrintData("outside limits",14);
  return -1;
 }

//determine starting direction and calculate 
//can change based on start and stop values
 if((stop-start)>0)
 {
  up=true;
  min_DAC = (int16_t) (round(start*(4096.0/3300))+2048);
  max_DAC = (int16_t) (round(stop*(4096.0/3300))+2048);
 }
 else
 {
  up=false;
  max_DAC = (int16_t) (round(start*(4096.0/3300))+2048); //Define the amount of current required to achieve required voltage
  min_DAC = (int16_t) (round(stop*(4096.0/3300))+2048);  //Define the amount of current required to achieve required voltage
 }
 //ramps is used to decide when it has performed reqiured number of sweeps and break the loop to end testing
 ramps = 2*scans;

 steps_per_sample = (uint16_t) (round(sample_rate*(4096.0/3300)));

 samples = 2*scans*((max_DAC-min_DAC)/steps_per_sample);
//Check if it is possible to compute based on supplied parameters
 if(samples > CV_MAX_DATAPOINTS)
 {
  lcdClear();
  lcdHome();
  lcdPrintData("too many data points",20);
  return -1;
 }

//2,000,000 [cycles/sec] * 1/slope [sec/mV] * 3300/4096 [mv/index]
 if(slope > 30)
 {
  step_time = (uint16_t) (round(2000000*(1.0/slope)*(3300.0/4096)));
  TIMER.CTRLA = TC_CLKSEL_DIV1_gc;
 }
 else
 {
  step_time = (uint16_t) (round(500000*(1.0/slope)*(3300.0/4096)));
  TIMER.CTRLA = TC_CLKSEL_DIV4_gc;
 }

//Define the current value to begin
 if(up)
  current_DAC = min_DAC;
 else
  current_DAC = max_DAC;

 i = 0;
 j = 0;
 steps_taken = 0;
 k = 0;

 for(k = 0; k < CV_BUFFER_SIZE; k++)
  result_buffer[k] = 0;

//change switches
 PORTE.OUTSET = PIN1_bm;  //switch0
 PORTE.OUTSET = PIN2_bm;  //switch2
 if(curr_range == RANGE_10UA)
  PORTE.OUTCLR = PIN3_bm;  //switch3
 else
  PORTE.OUTSET = PIN3_bm;  //switch3
//_delay_ms(50);
 PORTE.OUTCLR = PIN0_bm; //switch1

 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);
 _delay_ms(250);

while(1)
{
 //set DAC and trigger timer
 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);
 TIMER.CNT = 0;

//calculate next DAC value
 if(up)
  current_DAC++;
 else
  current_DAC--;

//decision making
 if(up && current_DAC >= max_DAC)
 {
  up = false; //switch to going down
  ramps--;
  if(ramps==0) //See if reached number of required sweeps
   break;
 }
 else if(!up && current_DAC <= min_DAC)
 {
  up = true;  //switch to going up
  ramps--;
  if(ramps==0) //See if reached number of required sweeps
   break;
 }

 current[i] = 0;
//ADC measurements
 while(TIMER.CNT<step_time) {
  ADC_Ch_Conversion_Start(&ADCA.CH1);
  while(!ADC_Ch_Conversion_Complete(&ADCA.CH1) && TIMER.CNT<step_time) {}

  if(current[i] == 0)
   current[i] = ADC_ResultCh_GetWord_Signed(&ADCA.CH1,ADC_OFFSET);
  else
   current[i] = (current[i] + ADC_ResultCh_GetWord_Signed(&ADCA.CH1,ADC_OFFSET)) >> 1;
 }

 steps_taken++;
 if(steps_taken >= steps_per_sample)
 {
  {
   
   n=i;
  }
  steps_taken = 0;
  i++;

 }
}
HiVal = current[n/2];
for (l=n/2;l<=n;l++)
{
  sum +=current[l];
  if (HiVal<current[l])
    HiVal = current[l];
}
mean = sum/(n/2);


for (l=n/2;l<=n;l++)
{
 st = mean-current[l];
 st2 = st*st;
 st3+=st2;
}
variance = st3/(n/2);
StDev = sqrt(variance);

//Change switches
 PORTE.OUTSET = PIN0_bm;  //switch1

 PORTE.OUTCLR = PIN1_bm;  //switch0
 PORTE.OUTCLR = PIN2_bm;  //switch2
 PORTE.OUTCLR = PIN3_bm;  //switch3
 current_DAC = 2048;
 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);

//start output to USB
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, CV);
 for(j = 0; j < 15; j++)
 {
  do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, name[j]);
 }
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, slope>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, slope);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, start>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, start);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, stop>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, stop);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, scans>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, scans);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, sample_rate>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, sample_rate);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, curr_range);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, i>>8);
 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
 USART_PutChar(&USARTC0, i);

 for(j = 0; j < i; j++)
 {
  do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, current[j]>>8);
  do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, current[j]);
 }

 do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0,CV);

 return i;
}

ISR(USARTC0_RXC_vect)
{
 uint8_t i,j,type;
 USART_RXComplete(&USART_data);
 if(USART_RXBufferData_Available(&USART_data))
  type = USART_RXBuffer_GetByte(&USART_data);
 else
  type = 0;
 //recieve profiles
 if(type == 'u')
 {
  for(i = 0; i < PROFILES_LENGTH; i++)
  {
   for(j = 0; j < 15; j++)
   {
    do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
    profiles[i].name[j] = USART_RXBuffer_GetByte(&USART_data);
   }
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].type = USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op1 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op1 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op2 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op2 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op3 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op3 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op4 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op4 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op5 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op5 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op6 = USART_RXBuffer_GetByte(&USART_data)<<8;
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].op6 |= USART_RXBuffer_GetByte(&USART_data);
   do{USART_RXComplete(&USART_data);} while(!USART_RXBufferData_Available(&USART_data));
   profiles[i].curr_range |= USART_RXBuffer_GetByte(&USART_data);
   //write to EEPROM
   eeprom_write_block((const void*)&(profiles[i]), (void*)&(profilesEE[i]), sizeof(profile));
  }
 }
//send profiles
 else if(type == 'd');
 {
  do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, 'z');
  for(i = 0; i < PROFILES_LENGTH; i++)
  {
   for(j = 0; j < 15; j++)
   {
    do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
    USART_PutChar(&USARTC0, profiles[i].name[j]);
   }
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].type);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op1>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op1);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op2>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op2);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op3>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op3);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op4>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op4);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op5>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op5);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op6>>8);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].op6);
   do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
   USART_PutChar(&USARTC0, profiles[i].curr_range);
  }
  do{} while(!USART_IsTXDataRegisterEmpty(&USARTC0));
  USART_PutChar(&USARTC0, 'z');
 }
}
