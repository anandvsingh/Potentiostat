#define TIMER TCC0

#define DAC_CONINTVAL DAC_CONINTVAL_1CLK_gc
#define DAC_REFRESH DAC_REFRESH_16CLK_gc
#define ADC_PRESCALER ADC_PRESCALER_DIV8_gc
#define ADC_OFFSET 0

#define EEPROM_SIZE 1024
#define PROFILES_LENGTH 11



#define CV_BUFFER_SIZE 16
#define CV_MAX_DATAPOINTS 1500

#define CV 1





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
float StDev;

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





 PORTA.DIRCLR = PIN4_bm;
 PORTA.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN5_bm;
 PORTA.PIN5CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN6_bm;
 PORTA.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
 PORTA.DIRCLR = PIN7_bm;
 PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;







 PORTE.DIRSET = PIN1_bm;
 PORTE.DIRSET = PIN0_bm;
 PORTE.DIRSET = PIN2_bm;
 PORTE.DIRSET = PIN3_bm;


 PORTE.OUTCLR = PIN1_bm;
 PORTE.OUTSET = PIN0_bm;
 PORTE.OUTCLR = PIN2_bm;
 PORTE.OUTCLR = PIN3_bm;





 DAC_DualChannel_Enable( &DACB,DAC_REFSEL_AVCC_gc,false,DAC_CONINTVAL,DAC_REFRESH);

 while (DAC_Channel_DataEmpty(&DACB, CH1) == false) {}
  DAC_Channel_Write(&DACB,2048,CH1);

 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,2048,CH0);





 PORTC.DIRSET = PIN3_bm;

 PORTC.DIRCLR = PIN2_bm;
 USART_InterruptDriver_Initialize(&USART_data, &USARTC0, USART_DREINTLVL_LO_gc);

 USART_Format_Set(&USARTC0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

 USART_RxdInterruptLevel_Set(&USARTC0, USART_RXCINTLVL_LO_gc);

 USART_Baudrate_Set(&USARTC0, 12 , 0);
 USART_Rx_Enable(&USARTC0);
 USART_Tx_Enable(&USARTC0);





 ADC_CalibrationValues_Load(&ADCA);

   ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);

 ADC_Prescaler_Config(&ADCA, ADC_PRESCALER);

 ADC_Reference_Config(&ADCA, ADC_REFSEL_VCC_gc);

 ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,ADC_CH_INPUTMODE_DIFF_gc,ADC_DRIVER_CH_GAIN_NONE);
 ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH1,ADC_CH_INPUTMODE_DIFF_gc,ADC_DRIVER_CH_GAIN_NONE);

 ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXNEG_PIN1_gc);
 ADC_Ch_InputMux_Config(&ADCA.CH1, ADC_CH_MUXPOS_PIN2_gc, ADC_CH_MUXNEG_PIN1_gc);

 ADC_Enable(&ADCA);
 lcdInit();
 lcdInit();
 lcdClear();
 lcdHome();
 lcdPrintData("CDAC Kolkata",12);
 _delay_ms(1000);




 status = PROFILE_SEL;
 profile_index = 1;
 profile_opt_index = OPT_START;
 profile_edit_index = 0;
 profile_edit_sel = EDIT_NOSEL;





 for(i = 0; i < PROFILES_LENGTH; i++) //Change to 1
  eeprom_read_block((void*)&(profiles[i]), (const void*)&(profilesEE[i]), sizeof(profile));




 PMIC.CTRL |= PMIC_LOLVLEX_bm;
 sei();

 strcpy(profiles[1].name,"CV #1          ");
 profiles[1].type = CV;
 profiles[1].op1 = 400;
 profiles[1].op2 = 0;
 profiles[1].op3 = 1000;
 profiles[1].op4 = 1;
 profiles[1].op5 = 4;
 profiles[1].op6 = 0;
 profiles[1].curr_range = RANGE_10UA;

 for(i = 0; i < PROFILES_LENGTH; i++) //change to 1
  eeprom_write_block((const void*)&(profiles[i]), (void*)&(profilesEE[i]), sizeof(profile));



 while(1)
 {

  if(status == PROFILE_SEL)
  {

   lcdClear();
   lcdHome();

      lcdPrintData("~",1);


     lcdPrintData(profiles[1].name, 15);

   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }

  else if(status == DISPLAY_RESULTS)
  {
   lcdClear();
   lcdHome();
   lcdPrintData("StDev = ",8);
   sprintf(Resultstr,"%4d",StDev);
   lcdPrintData(Resultstr,4);
   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}

  }
  else if(status == PROFILE_OPT)
  {

   lcdClear();
   lcdHome();


   lcdPrintData(profiles[profile_index].name, 15);


   lcdGotoXY(0,1);
   if(profile_opt_index == OPT_START)
    lcdPrintData("~",1);
   else
    lcdPrintData(" ",1);
   lcdPrintData("start",5);

   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }

  else if(status == PROFILE_TEST)
  {

   lcdClear();
   lcdHome();


   lcdPrintData(profiles[profile_index].name, 15);

   lcdGotoXY(0,1);
   lcdPrintData(" testing...",10);





      length = CV_test(profiles[profile_index].name, profiles[profile_index].op1, profiles[profile_index].op2, profiles[profile_index].op3, profiles[profile_index].op4, profiles[profile_index].op5, profiles[profile_index].curr_range);

   if(length == -1)
   {
    _delay_ms(1000);
    status = PROFILE_EDIT;
   }
   else
    status = PROFILE_RESULTS;
  }

  else if (status == PROFILE_RESULTS)
  {

   lcdClear();
   lcdHome();


   lcdPrintData(profiles[profile_index].name, 15);


   lcdGotoXY(0,1);
   lcdPrintData("Test Complete",13);



   while(buttonHandler(profiles,&status,&profile_index,&profile_opt_index,&profile_edit_index,&profile_edit_sel,&length)!=1) {}
  }

 }

 return 0;


}

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

 if(bit_is_set(PORTA.IN,4))
  dir = RIGHT;
 else if(bit_is_set(PORTA.IN,5))
  dir = DOWN;
 else if(bit_is_set(PORTA.IN,6))
  dir = LEFT;
 else if(bit_is_set(PORTA.IN,7))
  dir = UP;
 }




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
  if (dir==RIGHT)
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
 int sum = 0,n,l,flag=0;
 float mean,st,st2,st3;
 unsigned long variance;
 bool up;

 int16_t current_DAC, min_DAC, max_DAC, zero_DAC;


 int16_t current[CV_MAX_DATAPOINTS];
 int16_t result_buffer[CV_BUFFER_SIZE];


 if(start<-1600 || start>1600 || stop<-1600 || stop>1600 || slope>9000 || slope<10 || sample_rate<1 || sample_rate>1600)
 {
  lcdClear();
  lcdHome();
  lcdPrintData("outside limits",14);
  return -1;
 }


 //if((stop-start)>0)
 {
  up=true;
  zero_DAC = (int16_t) (round(start*(4096.0/3300))+2048);
  max_DAC = (int16_t) (round(stop*(4096.0/3300))+2048);
  min_DAC = (int16_t) (round(-1000*(4096.0/3300))+2048);
 }
 /*else
 {
  up=false;
  max_DAC = (int16_t) (round(start*(4096.0/3300))+2048);
  min_DAC = (int16_t) (round(stop*(4096.0/3300))+2048);
 }*/

 ramps = 2*scans;

 steps_per_sample = (uint16_t) (round(sample_rate*(4096.0/3300)));

 samples = 2*scans*((max_DAC-min_DAC)/steps_per_sample);

 if(samples > CV_MAX_DATAPOINTS)
 {
  lcdClear();
  lcdHome();
  lcdPrintData("too many data points",20);
  return -1;
 }


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


 //if(up)
  current_DAC = max_DAC;
 //else
  //current_DAC = max_DAC;

 i = 0;
 j = 0;
 steps_taken = 0;
 k = 0;

 for(k = 0; k < CV_BUFFER_SIZE; k++)
  result_buffer[k] = 0;


 PORTE.OUTSET = PIN1_bm;
 PORTE.OUTSET = PIN2_bm;
 if(curr_range == RANGE_10UA)
  PORTE.OUTCLR = PIN3_bm;
 else
  PORTE.OUTSET = PIN3_bm;

 PORTE.OUTCLR = PIN0_bm;

 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);
 _delay_ms(250);

while(1)
{

 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);
 TIMER.CNT = 0;


 /*if(up)
  current_DAC++;
 else
  current_DAC--;


 if(up && current_DAC >= max_DAC)
 {
  up = false;
  ramps--;
  //if(ramps==0)
   //break;
  flag=1;
 }
 else if(!up && current_DAC <= min_DAC)
 {
  up = true;
  ramps--;
  //if(ramps==0)
   //break;
 }
else if(up && current_DAC >= zero_DAC && flag==1)
{
  break;
}
*/

if (i==1000)
{
  break;
}
 current[i] = 0;

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
   sum+=current[i];
   n=i;
  }
  steps_taken = 0;
  i++;

 }
}
mean = sum/n;
for (l=0;l<=n;l++)
{
 st = mean-current[l];
 st2 = st*st;
 st3+=st2;
}
variance = st3/n;
StDev = sqrt(variance);



 PORTE.OUTSET = PIN0_bm;

 PORTE.OUTCLR = PIN1_bm;
 PORTE.OUTCLR = PIN2_bm;
 PORTE.OUTCLR = PIN3_bm;
 current_DAC = 2048;
 while (DAC_Channel_DataEmpty(&DACB, CH0) == false) {}
  DAC_Channel_Write(&DACB,current_DAC,CH0);


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

   eeprom_write_block((const void*)&(profiles[i]), (void*)&(profilesEE[i]), sizeof(profile));
  }
 }

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