/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "lunar_data.h"
//E B7
//C B5
//D B6
//G B4
//F B3
//A B2
//B B1
//0b00101000


uint8_t led7_b[10]     = {0b00010000,0b11011100,0b00101000,0b10001000,0b11000100,0b10000010,0b00000010,0b11011000,0b00000000,0b10000000};


//B B7
//A B6
//F B5
//G B4
//C B3
//D B2

//E B0
uint8_t led7_d[10] = {0b00010000,0b01110111,0b00101000,0b00100001,0b01000101,0b10000001,0b10000000,0b00110111,0b00000000,0b00000001};

//E B7
//D B6
//C B5
//G B4
//F B3
//A B2

//B B0
uint8_t led7_d_1[10] = {0b00010000,0b11011110,0b00101000,0b10001000,0b11000100,0b10000001,0b00000001,0b11011010,0b00000000,0b10000000};
//G1 MIN DV
//E3 MIN CHUC
//C7 HOUR DV
//A4 HOUR CHUC
//G0 AM NGAY CHUC
//C6 AM NGAY DV
//C3 AM THANG CHUC
//E5 AM THANG DV
//C4 TEMPERATURE CHUC
//C2 TEMPERATURE DV
//E7 THU
//A6 DUONG NGAY CHUC
//A5 DUONG NGAY DV
//A1 DUONG THANG CHUC
//C5 DUONG THANG DV
//C1 DUONG NAM CHUC
//A3 DUONG NAM DV
//A2 SECOND
#define DAY 0
#define DATE_X 1
#define DATE_Y 2
#define MONTH_X 3
#define MONTH_Y 4
#define YEAR_X 5
#define YEAR_Y 6
#define HOUR_X 7
#define HOUR_Y 8
#define MIN_X 9
#define MIN_Y 10
#define AM_D_X 11
#define AM_D_Y 12
#define AM_M_X 13
#define AM_M_Y 14
#define TEMP_X 15
#define TEMP_Y 16
#define DOT_SEC 17
//G1 MIN DV
//G0 AM NGAY CHUC

//E7 THU
//E5 AM THANG DV
//E3 MIN CHUC

//A6 DUONG NGAY CHUC
//A5 DUONG NGAY DV
//A4 HOUR CHUC
//A3 DUONG NAM DV
//A2 SECOND
//A1 DUONG THANG CHUC

//C7 HOUR DV
//C6 AM NGAY DV
//C5 DUONG THANG DV
//C4 TEMPERATURE CHUC
//C3 AM THANG CHUC
//C2 TEMPERATURE DV
//C1 DUONG NAM CHUC	

uint8_t DIG[18] = {0};
uint8_t seg = 1, value = 99, temp_ir = 0;
uint8_t data_time_display[17] = {1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1, 2, 3};
uint8_t data_time_on_pairs[9] = {150, 150, 150, 150, 150, 150, 150, 150, 20};
uint8_t mode_value = 0;
double temperature = 0;
//IR
uint8_t done = 0, is_repeat = 0;
int16_t count_ms1 = 0, count_bit_ir = 0, start_status = -1;
uint32_t code_ir = 0;
#define EDIT_MODE 0x40BDA25D //increase value
#define KEY_INC 0x40BDBB44 //increase value
#define KEY_DEC 0x40BD31CE //decrease value
#define KEY_PAGE_RIGHT 0x40BD59A6 //Page right
#define KEY_PAGE_LEFT 0x40BDB34C //page left
//IR end init

//I2C
#define I2C_READ	1
#define I2C_WRITE	0
uint8_t reg = 0;
uint8_t data_time[9] = {1};
//I2C end init
void check_button(void);
void delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
INTERRUPT void TIM4_UPD_OVF_IRQHandler(void)
{
	switch(seg)
	{
		case 1:
		{
			TIM4->ARR  = data_time_on_pairs[0];
			GPIOB->ODR  =    led7_b[data_time_display[DATE_X]];

			GPIOD->ODR  =    led7_d[data_time_display[MIN_X]];
			
			GPIOA->ODR = 0b01000100 & (~(DIG[DATE_X] << 6));
			GPIOC->ODR = 0b00000000;
			GPIOE->ODR = 0b00001000 & (~(DIG[MIN_X] << 3));
			GPIOG->ODR = 0b00000000;
			break;
		}					
		case 3:
		{
			TIM4->ARR  = data_time_on_pairs[1];
			GPIOB->ODR  =    led7_b[data_time_display[DATE_Y]];

			GPIOD->ODR  =    led7_d[data_time_display[MIN_Y]];
			
			GPIOA->ODR = 0b00100100 & (~(DIG[DATE_Y] << 5));
			GPIOC->ODR = 0b00000000;		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000010 & (~(DIG[MIN_Y] << 1));
			break;
		}						
		case 5:
		{
			TIM4->ARR  = data_time_on_pairs[2];
			GPIOB->ODR  =    led7_b[data_time_display[MONTH_X]];

			GPIOD->ODR  =    led7_d_1[data_time_display[AM_D_X]];
						
			GPIOA->ODR = 0b00000110 & (~(DIG[MONTH_X] << 1));
			GPIOC->ODR = 0b00000000;	
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000001 & (~(DIG[AM_D_X] << 0));
			break;
		}
		case 7:
		{
			TIM4->ARR  = data_time_on_pairs[3];
			GPIOB->ODR  =    led7_b[data_time_display[MONTH_Y]];

			GPIOD->ODR  =    led7_d_1[data_time_display[AM_D_Y]];
						
			GPIOA->ODR = 0b00000100;
			GPIOC->ODR = 0b01100000& (~(DIG[MONTH_Y] << 5)) & (~(DIG[AM_D_Y] << 6));
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 9:
		{
			TIM4->ARR  = data_time_on_pairs[4];
			GPIOB->ODR  =    led7_b[data_time_display[YEAR_X]];

			GPIOD->ODR  =    led7_d_1[data_time_display[AM_M_X]];		
			
			GPIOA->ODR = 0b00000100;
			GPIOC->ODR = 0b00001010 & (~(DIG[YEAR_X] << 1)) & (~(DIG[AM_M_X] << 3));
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 11:
		{
			TIM4->ARR  = data_time_on_pairs[5];
			GPIOB->ODR  =    led7_b[data_time_display[YEAR_Y]];
			GPIOD->ODR  =    led7_d_1[data_time_display[AM_M_Y]];
			
			GPIOA->ODR = 0b00001100 & (~(DIG[YEAR_Y] << 3));
			GPIOC->ODR = 0b00000000;
			GPIOE->ODR = 0b00100000 & (~(DIG[AM_M_Y] << 5));
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 13:
		{
			TIM4->ARR  = data_time_on_pairs[6];
			GPIOB->ODR  = led7_b[data_time_display[HOUR_X]];//data_time_display[HOUR_X]

			GPIOD->ODR  = led7_d_1[data_time_display[TEMP_X]];			
						
			GPIOA->ODR = 0b00010100 & (~(DIG[HOUR_X] << 4));
			GPIOC->ODR = 0b00010000 & (~(DIG[TEMP_X] << 4));
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 15:
		{
			TIM4->ARR  = data_time_on_pairs[7];
			GPIOB->ODR  = led7_b[data_time_display[DAY]];

			GPIOD->ODR  = led7_d[data_time_display[HOUR_Y]];	
						
			GPIOA->ODR = 0b00000100;
			GPIOC->ODR = 0b10000000 & (~(DIG[HOUR_Y] << 7));
			GPIOE->ODR = 0b10000000 & (~(DIG[DAY] << 7));
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 17:
		{
			TIM4->ARR  = data_time_on_pairs[8];
			GPIOD->ODR  = led7_d_1[data_time_display[TEMP_Y]];
						
			GPIOA->ODR = 0b00000100 & (~(DIG[DOT_SEC] << 2));			
			GPIOC->ODR = 0b00000100;
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 2: // 0
		case 4: // 1
		case 6: // 2
		case 8: //3
		case 10: //4
		case 12: //5
		case 14: //6
		case 16: //7
		case 18: //8
		{
			GPIOB->ODR = 0xFF;
			GPIOD->ODR = 0xFF;
			//check_button();
			TIM4->ARR  = 255 - data_time_on_pairs[(seg/2) - 1];
			GPIOA->ODR = 0b00000100;
			
			GPIOC->ODR = 0b00000000;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
	}

	//G1 MIN DV
	//G0 AM NGAY CHUC
	//GPIOG->ODR = 0b00000000 | (DIG[MIN_Y] << 1) | (DIG[AM_D_X] << 0);
	
	//E7 THU
	//E5 AM THANG DV
	//E3 MIN CHUC
	//GPIOG->ODR = 0b00000000 | (DIG[DAY] << 7) | (DIG[AM_M_Y] << 5) | (DIG[MIN_X] << 3);
	
	//A6 DUONG NGAY CHUC
	//A5 DUONG NGAY DV
	//A4 HOUR CHUC
	//A3 DUONG NAM DV
	//A2 SECOND
	//A1 DUONG THANG CHUC
	//GPIOA->ODR = 0b00000000 | (DIG[DATE_X] << 6) | (DIG[DATE_Y] << 5) | (DIG[HOUR_X] << 4) | (DIG[YEAR_Y] << 3) | (DIG[DOT_SEC] << 2) | (DIG[MONTH_X] << 1);
	
	//C7 HOUR DV
	//C6 AM NGAY DV
	//C5 DUONG THANG DV
	//C4 TEMPERATURE CHUC
	//C3 AM THANG CHUC
	//C2 TEMPERATURE DV
	//C1 DUONG NAM CHUC	
	//GPIOC->ODR = 0b00000000 | (DIG[HOUR_X] << 7) | (DIG[AM_D_Y] << 6) | (DIG[MONTH_Y] << 5) | (DIG[TEMP_X] << 4) | (DIG[AM_M_X] << 3) | (DIG[TEMP_Y] << 2) | (DIG[YEAR_X] << 1);	

	seg++;
	if(seg > 19)
	{
		seg = 1;
	}
	TIM4->SR1 &= ~(1 << 0);
}
void reset_to_new_cmd(void){
	count_ms1=0;
	start_status=-1;
	count_bit_ir=0;
	code_ir = 0;
	done = 0;
}
INTERRUPT void TIM2_UPD_OVF_IRQHandler(void){
	
	if((start_status == 0) || (done == 1) || (start_status == 1)) {
		reset_to_new_cmd();
		is_repeat = 0;
	}
	//mode = 0;
	//clear trigger
	//data_time_display[1] = 0; data_time_display[2] = 0;
	//15625/s
	TIM2->SR1 = 0b00000000;
}
INTERRUPT void EXTI_PORTE_IRQHandler()
{
	if((GPIOE->IDR & 0x01) == 0){	
		switch(start_status){
			case -1:
				TIM2->CNTRL = 0;
				TIM2->CNTRH = 0xFF;
				start_status = 0;
				break;
			case 0:
				count_ms1 = TIM2->CNTRL;
				TIM2->CNTRH = 0xFF;
				if(((count_ms1)>=200)&&((count_ms1)<=230))//10ms-14ms start, repeat 9ms+ 2.25ms = 11.25
				{
					count_bit_ir=0;
					start_status=1;
					count_ms1=0;
					TIM2->CNTRL = 0;
				} else if(count_ms1 <= 180)//repeat 9ms+ 2.25ms = 11.25
				{
					/*
					count_bit_ir=0;
					start_status=-1;
					count_ms1=0;
					done = 1;
					data = 0xFFFFFFFF;
					*/
					//define repeate cycle timeout 240ms count 3750
					TIM2->CNTRH = 0xF1;
					TIM2->CNTRL = 0x59;
					is_repeat++;
					//value = is_repeat;
				} else if(count_ms1>226)/// error detect xung start
				{
					count_ms1=0;
					start_status=-1;
					count_bit_ir=0;
					is_repeat = 0;
					code_ir=0;
				}
				break;
			case 1:
				TIM2->CNTRH = 0xFF;
				count_ms1 = TIM2->CNTRL;
				//value = count_ms1;
				TIM2->CNTRL = 0;
				if((count_bit_ir>=0)&&(count_bit_ir<=31)){
					//if(count_bit_ir == 1) 
					value = count_bit_ir;
					if((count_ms1>=20)&&(count_ms1<=47))//2ms->3ms //detect logic 1.
					{
						code_ir|=(uint32_t)1<<(31-count_bit_ir);
						count_ms1=0;
					}
					else if(count_ms1<20)//nho hon <2ms //detect logic 0.
					{
						count_ms1=0;
					}
					else //error data reset all
					{
						//value = count_bit_ir+500;	
						is_repeat = 0;
						count_ms1=0;
						start_status=-1;
						count_bit_ir=0;
						code_ir=0;
					}
					count_bit_ir++;	
					//value =	count_bit_ir;	
								
				}
				
				if(count_bit_ir == 32) //reset sau khi detect 32 bit data
				{
					//value = (data >> 24) & 0xFF;
					//value = (data >> 16) & 0xFF;
					//value = (data >> 8) & 0xFF;
					//value = (data >> 0) & 0xFF;	
					//temp_ir = count_bit_ir;
					TIM2->CNTRL = 0xCB;
					TIM2->CNTRH = 0xF3;	
					is_repeat = 0;
					done = 1;
					
				}
				break;
		}
	}
}
void timer1_init(void){
	TIM1->CR1 = 0x00;
	TIM1->PSCRH = 0x3E; //3E
	TIM1->PSCRL = 0x7F; //7F
	TIM1->CR1 = 0x01;
}
void timer4_init(void) {
	// CK_PSC (internal fMASTER) is divided by the prescaler value.
	TIM4->PSCR = 7;
	TIM4->ARR = 200;
	// Enable update interrupt for timer 4
	TIM4->IER |= (1 << 0);
	// Clear timer interrupt flag
	TIM4->SR1 &= ~(1 << 0);
	// Precalculated value
	//TIM4_CNTR = 0xFF - 126;
	// Enable timer 4
	TIM4->CR1 |= (1 << 0);
	//Enable auto-reload
	TIM4->CR1 |= (0 << 7);
}
void timer2_init(void){
TIM2->PSCR = 0b00001010;       //  Prescaler = 1.
//TIM2->ARRH = 0x00;       //  High byte of 8,000.
//TIM2->ARRL = 0x00;       //  Low byte of 8,000.
TIM2->IER = 0b00000001;       //  Enable the update interrupts.
TIM2->CR1 = 0b00000001;       //  Finally enable the timer.
}

void i2c_init(void){
	I2C->CR1 = 0;
	
	I2C->FREQR = 16;             //  Set the internal clock frequency (MHz).
	I2C->CCRH = 0x0F;
	I2C->CCRL = 0xFF;
	
	//  Set the address mode of this device.
	I2C->OARH = 0b01000000;
	
	//  Setup the bus characteristics.
	I2C->TRISER = 17;
	
	//  Configuration complete so turn the peripheral on.
	I2C->CR1 = 0x01;
}
uint8_t step_treo = 0;
uint8_t test_i2c(uint8_t offset){
	uint32_t wdt = 0;
	//  Enter master mode.
	I2C->CR2 = 0b00000101;
	//wait START generate complete
	step_treo = 0;
	while (((I2C->CR2) & 0x01) == 0x01);
	//clear SB bit after START complete
	step_treo = 1;
	while(((I2C->SR1) & 0x01) == 0x00);
	//write data to bus
	I2C->DR = (0x68 << 1) | I2C_WRITE;
	//wait ADDR bit is set
	step_treo = 2;
	while (((I2C->SR1 >> 1) & 0x01) == 0x00);
	reg = I2C->SR1;
	reg = I2C->SR3;
	reg = I2C->SR3;
	//end address phase
	
	I2C->DR = offset;
	//wait TXE bit is set
	step_treo = 3;
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	//generate STOP
	I2C->CR2 = 0b00000110;
	//wait STOP generate complete
	step_treo = 4;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	//generate START
	I2C->CR2 = 0b00000101;
	//wait START generate complete
	step_treo = 5;

	while (((I2C->CR2) & 0x01) == 0x01);
	//clear SB bit after START complete
	step_treo = 6;

	while(((I2C->SR1) & 0x01) == 0x00);
	//write data to bus
	I2C->DR = (0x68 << 1) | I2C_READ;
	
	//wait ADDR bit is set
	step_treo = 7;
	while (((I2C->SR1 >> 1) & 0x01) == 0x00);
	//set NACK after receive one byte
	I2C->CR2 = 0b00000000;
	reg = I2C->SR1;
	reg = I2C->SR3;
	

	//wait ADDR bit is cleared
	step_treo = 8;
	while (((I2C->SR1 >> 1) & 0x01) == 0x01);
	//set STOP after receive done
	I2C->CR2 = 0b00000010;
	reg = I2C->DR;
	reg = I2C->SR3;
	step_treo = 9;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	step_treo = 10;
	//while (((I2C->SR1 >> 6) & 0x01) == 0x00);
	reg = I2C->DR;
	return reg;
}
uint8_t bcd2dec(uint8_t num)
{
	return ((num/16 * 10) + (num % 16));
}
uint8_t dec2bcd(uint8_t num)
{
	return ((num/10 * 16) + (num % 10));
}
//setTime(8, 30, 0, 2, 12, 10, 29);
void setTime(uint8_t hr, uint8_t min, uint8_t sec, uint8_t wd, uint8_t d, uint8_t mth, uint8_t yr)
{
	//  Enter master mode.
	I2C->CR2 = 0b00000101;
	//wait START generate complete
	while (((I2C->CR2) & 0x01) == 0x01);
	//clear SB bit after START complete
	while(((I2C->SR1) & 0x01) == 0x00);
	//write data to bus
	I2C->DR = (0x68 << 1) | I2C_WRITE;
	//wait ADDR bit is set
	while (((I2C->SR1 >> 1) & 0x01) == 0x00);
	reg = I2C->SR1;
	reg = I2C->SR3;
	reg = I2C->SR3;
	//end address phase
	
	//set pointer to 0
	I2C->DR = 0x00;
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(sec);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(min);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(hr);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(wd);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(d);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(mth);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	I2C->DR = dec2bcd(yr);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	//generate STOP and wait
	I2C->CR2 = 0b00000110;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	
}
void init_time_display(void){
	data_time[0] = test_i2c(0x00);
	data_time[1] = test_i2c(0x01);
	data_time[2] = test_i2c(0x02);
	data_time[3] = test_i2c(0x03);
	data_time[4] = test_i2c(0x04);
	data_time[5] = test_i2c(0x05);
	data_time[6] = test_i2c(0x06);
}
void lunar_convert(void){
	uint8_t da,db,day,mon,year;
	uint8_t lmon;

	day = bcd2dec(data_time[4]);
	mon = bcd2dec(data_time[5]);
	year = bcd2dec(data_time[6]);
	da = ALdauthangDL[year - 10][mon-1];
	db = DLdauthangAL[year - 10][mon-1];	

	if(db <= day){
		data_time[7] = day-db+1;
		data_time[8] = thangALdauthangAL[year-10][mon-1];				
	}else {
		data_time[7] = day+da-1;
		data_time[8] = thangALdauthangDL[year-10][mon-1];		
	}
}
void init_output(void){
	GPIOA->DDR = 0b01111110;//
	GPIOB->DDR = 0b11111110;//
	GPIOC->DDR = 0b11111110;//
	GPIOD->DDR = 0b11111111;//
	GPIOE->DDR = 0b10101000;//
	GPIOG->DDR = 0b00000011;//
	
	GPIOA->CR1 = 0b01111110;
	GPIOB->CR1 = 0b11111110;
	GPIOC->CR1 = 0b11111110;
	GPIOD->CR1 = 0b11111111;
	GPIOE->CR1 = 0b11101000;
	GPIOG->CR1 = 0b00000011;
							
	GPIOA->CR2 = 0b01111110;
	GPIOB->CR2 = 0b11111110;
	GPIOC->CR2 = 0b11111110;
	GPIOD->CR2 = 0b11111111;
	GPIOE->CR2 = 0b10101001;
	GPIOG->CR2 = 0b00000011;
}
void init_input(void){
	GPIOA->DDR = 0b01111110;//
	GPIOB->DDR = 0b11111110;//
	GPIOC->DDR = 0b11101110;//
	GPIOD->DDR = 0b11111111;//
	GPIOE->DDR = 0b10001000;//
	GPIOG->DDR = 0b00000011;//
	
	GPIOA->CR1 = 0b01111110;
	GPIOB->CR1 = 0b11111110;
	GPIOC->CR1 = 0b11111110;
	GPIOD->CR1 = 0b11111111;
	GPIOE->CR1 = 0b11101000;
	GPIOG->CR1 = 0b00000011;
							
	GPIOA->CR2 = 0b01111110;
	GPIOB->CR2 = 0b11111110;
	GPIOC->CR2 = 0b11101110;
	GPIOD->CR2 = 0b11111111;
	GPIOE->CR2 = 0b10001001;
	GPIOG->CR2 = 0b00000011;
	GPIOD->ODR = 0xFF;
	
}
uint16_t timer1_value = 0, timer_second = 1000, timer_button = 0, timer_check_button = 0;
uint8_t button_set = 0, button_up = 0, button_down = 0, temp = 0;
void check_button(void){
	init_input();
	if(( ((GPIOE->IDR >> 6) & 0x01) == 0x00) && (button_set == 0)){
		timer_button = TIM1->CNTRH<<8;
		timer_button |= TIM1->CNTRL;
		button_set = 1;
	} else {
		button_set = 0;
	}
	if((button_set) && (timer1_value - timer_button > 1000)){
		button_set = 0;
		code_ir = EDIT_MODE;
		done = 1;
		if(mode_value == 0) init_input();
		while(((GPIOE->IDR >> 6) & 0x01) == 0x00) temp = 1;
	}
	if(( ((GPIOE->IDR >> 5) & 0x01) == 0x00) && (button_up == 0) && (mode_value != 0)){
		timer_button = TIM1->CNTRH<<8;
		timer_button |= TIM1->CNTRL;
		button_up = 1;
	} else {
		button_up = 0;
	}
	if((button_up) && (timer1_value - timer_button > 1000) && (mode_value != 0)){
		button_up = 0;
		code_ir = KEY_INC;
		done = 1;
		while(((GPIOE->IDR >> 5) & 0x01) == 0x00) temp = 2;
	}
	if(( ((GPIOC->IDR >> 4) & 0x01) == 0x00) && (button_down == 0) && (mode_value != 0)){
		timer_button = TIM1->CNTRH<<8;
		timer_button |= TIM1->CNTRL;
		button_down = 1;
	} else {
		button_down = 0;
	}
	if((button_down) && (timer1_value - timer_button > 1000) && (mode_value != 0)){
		button_down = 0;
		code_ir = KEY_DEC;
		done = 1;
		while(((GPIOC->IDR >> 4) & 0x01) == 0x00);
	}
	init_output();
}
void SetupADC(void)
{
	ADC1->CR1 |= (1 < 0);  //  Turn ADC on, note a second set is required to start the conversion.
	ADC1->CSR  = 0x00;      // AIN0.
	ADC1->CR2  |= (1 << 3);  //  Data is right aligned.
}
const double BALANCE_RESISTOR   = 10000.0;
const double MAX_ADC            = 1023.0;
const double BETA               = 3435l;
const double ROOM_TEMP          = 298.15;   // room temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 10000.0;
double ln(double x)
{
    double old_sum = 0.0;
    double xmlxpl = (x - 1) / (x + 1);
    double xmlxpl_2 = xmlxpl * xmlxpl;
    double denom = 1.0;
    double frac = xmlxpl;
    double term = frac;                 // denom start from 1.0
    double sum = term;

    while ( sum != old_sum )
    {
        old_sum = sum;
        denom += 2.0;
        frac *= xmlxpl_2;
        sum += frac / denom;
    }
    return 2.0 * sum;
}
#define LN10 2.3025850929940456840179914546844

double log10( double x ) {
    return ln(x) / LN10;    
}
double adc_temperature(double adcAverage){
	double rThermistor = 0;            // Holds thermistor resistance value
  double tKelvin     = 0;            // Holds calculated temperature
  double tCelsius    = 0;            // Hold temperature in celsius
  //double adcAverage  = 0;            // Holds the average voltage measurement
	
	rThermistor = BALANCE_RESISTOR / ( (MAX_ADC / adcAverage) - 1);
	tKelvin = (BETA * ROOM_TEMP) / 
            (BETA + (ROOM_TEMP * log10(rThermistor / RESISTOR_ROOM_TEMP)));
	tCelsius = tKelvin - 273.15;  // convert kelvin to celsius 

  return rThermistor;    // Return the temperature in Celsius
}
double read_adc(void){
	unsigned char low, high;
	low = ADC1->DRL;            //    Extract the ADC reading.
	high = ADC1->DRH;
	return (double) (1023.0 - ((high * 256.0) + low));
}
main()
{
	
	CLK->CKDIVR = 0x00; // Set the frequency to 16 MHz
	delay(1000);
	init_output();
	timer1_init();
	timer4_init();
	timer2_init();
	//SetupADC();
	//i2c_init();
	EXTI->CR2 = 0b00000010;
	//ITC->ISPR2 = 0b00111111;
	//ITC->ISPR6 = 0b01111111;
	enableInterrupts();
	//setTime(13, 35, 00, 4, 28, 10, 20);
	//test_i2c(0x00);
	//init_time_display();
	while (1){
		timer1_value = TIM1->CNTRH<<8;
		timer1_value |= TIM1->CNTRL;
		timer_button = timer1_value; 
		//GPIOA->ODR = 0b01000100;//
		//GPIOB->ODR = led7_b[8];// led data left from second
		//GPIOC->ODR = 0b00000000;//
		//GPIOD->ODR = led7_d[8];// led data right from second
		//GPIOE->ODR = 0b00001000;//
		//GPIOG->ODR = 0b00000000;//
		
		if((timer1_value - timer_second > 500) && (mode_value == 0)){
			timer_second = timer1_value;
			if(DIG[DOT_SEC] == 0) {
				DIG[DOT_SEC] = 1;
				//ADC1->CR1 = 0b00000001;
			} else {
				DIG[DOT_SEC] = 0;
				
				//temperature = adc_temperature(511.0);

			}
			//data_time[0] = test_i2c(0x00);
			if(data_time[0] == 0){
				//data_time[1] = test_i2c(0x01);
				//data_time[2] = test_i2c(0x02);
				//data_time[3] = test_i2c(0x03);
				//data_time[4] = test_i2c(0x04);
				//data_time[5] = test_i2c(0x05);
				//data_time[6] = test_i2c(0x06);
				
			}
			
			data_time_display[DAY] = bcd2dec(data_time[3]);
			
			data_time_display[HOUR_X] = bcd2dec(data_time[2]) / 10;
			data_time_display[HOUR_Y] = bcd2dec(data_time[2]) % 10;
			data_time_display[MIN_X] = bcd2dec(data_time[1]) / 10;
			data_time_display[MIN_Y] = bcd2dec(data_time[1]) % 10;
			
			data_time_display[DATE_X] = bcd2dec(data_time[4]) / 10;
			data_time_display[DATE_Y] = bcd2dec(data_time[4]) % 10;
			
			data_time_display[MONTH_X] = bcd2dec(data_time[5]) / 10;
			data_time_display[MONTH_Y] = bcd2dec(data_time[5]) % 10;
			
			data_time_display[YEAR_X] = bcd2dec(data_time[6]) / 10;
			data_time_display[YEAR_Y] = bcd2dec(data_time[6]) % 10;
			
			lunar_convert();
			data_time_display[AM_D_X] = data_time[7] / 10;
			data_time_display[AM_D_Y] = data_time[7] % 10;
			
			data_time_display[AM_M_X] = data_time[8] / 10;
			data_time_display[AM_M_Y] = data_time[8] % 10;
			
			
			
		}

		
		if(done == 1){
			if(code_ir == EDIT_MODE){ 
				mode_value++; 
				if(mode_value > 7) mode_value = 0;
				reset_to_new_cmd();
				switch(mode_value){
					case 0:
						DIG[DAY] = 0; 
						DIG[DATE_X] = 0;DIG[DATE_Y] = 0;
						DIG[MONTH_X] = 0;DIG[MONTH_Y] = 0;
						DIG[YEAR_X] = 0;DIG[YEAR_Y] = 0;
						DIG[HOUR_X] = 0;DIG[HOUR_Y] = 0;
						DIG[MIN_X] = 0;DIG[MIN_Y] = 0;
						DIG[AM_D_X] = 0;DIG[AM_D_Y] = 0;
						DIG[AM_M_X] = 0;DIG[AM_M_Y] = 0;
						DIG[TEMP_X] = 0;DIG[TEMP_Y] = 0;
						break;
					case 1:
						DIG[DAY] = 0; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 1;DIG[DATE_Y] = 1;
						DIG[MONTH_X] = 1;DIG[MONTH_Y] = 1;
						DIG[YEAR_X] = 1;DIG[YEAR_Y] = 1;
						DIG[HOUR_X] = 1;DIG[HOUR_Y] = 1;
						DIG[MIN_X] = 1;DIG[MIN_Y] = 1;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						data_time[1] = bcd2dec(data_time[1]);
						data_time[2] = bcd2dec(data_time[2]);
						data_time[3] = bcd2dec(data_time[3]);
						data_time[4] = bcd2dec(data_time[4]);
						data_time[5] = bcd2dec(data_time[5]);
						data_time[6] = bcd2dec(data_time[6]);
						break;
					case 2:
						DIG[DAY] = 1; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 0;DIG[DATE_Y] = 0;
						DIG[MONTH_X] = 1;DIG[MONTH_Y] = 1;
						DIG[YEAR_X] = 1;DIG[YEAR_Y] = 1;
						DIG[HOUR_X] = 1;DIG[HOUR_Y] = 1;
						DIG[MIN_X] = 1;DIG[MIN_Y] = 1;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						break;
					case 3:
						DIG[DAY] = 1; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 1;DIG[DATE_Y] = 1;
						DIG[MONTH_X] = 0;DIG[MONTH_Y] = 0;
						DIG[YEAR_X] = 1;DIG[YEAR_Y] = 1;
						DIG[HOUR_X] = 1;DIG[HOUR_Y] = 1;
						DIG[MIN_X] = 1;DIG[MIN_Y] = 1;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						break;
					case 4:
						DIG[DAY] = 1; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 1;DIG[DATE_Y] = 1;
						DIG[MONTH_X] = 1;DIG[MONTH_Y] = 1;
						DIG[YEAR_X] = 0;DIG[YEAR_Y] = 0;
						DIG[HOUR_X] = 1;DIG[HOUR_Y] = 1;
						DIG[MIN_X] = 1;DIG[MIN_Y] = 1;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						break;
					case 5:
						DIG[DAY] = 1; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 1;DIG[DATE_Y] = 1;
						DIG[MONTH_X] = 1;DIG[MONTH_Y] = 1;
						DIG[YEAR_X] = 1;DIG[YEAR_Y] = 1;
						DIG[HOUR_X] = 0;DIG[HOUR_Y] = 0;
						DIG[MIN_X] = 1;DIG[MIN_Y] = 1;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						break;
					case 6:
						DIG[DAY] = 1; DIG[DOT_SEC] = 0;
						DIG[DATE_X] = 1;DIG[DATE_Y] = 1;
						DIG[MONTH_X] = 1;DIG[MONTH_Y] = 1;
						DIG[YEAR_X] = 1;DIG[YEAR_Y] = 1;
						DIG[HOUR_X] = 1;DIG[HOUR_Y] = 1;
						DIG[MIN_X] = 0;DIG[MIN_Y] = 0;
						DIG[AM_D_X] = 1;DIG[AM_D_Y] = 1;
						DIG[AM_M_X] = 1;DIG[AM_M_Y] = 1;
						DIG[TEMP_X] = 1;DIG[TEMP_Y] = 1;
						break;
					case 7:
						mode_value = 0;
						setTime(data_time[2], data_time[1], 00, data_time[3], data_time[4], data_time[5], data_time[6]);
						DIG[DAY] = 0; 
						DIG[DATE_X] = 0;DIG[DATE_Y] = 0;
						DIG[MONTH_X] = 0;DIG[MONTH_Y] = 0;
						DIG[YEAR_X] = 0;DIG[YEAR_Y] = 0;
						DIG[HOUR_X] = 0;DIG[HOUR_Y] = 0;
						DIG[MIN_X] = 0;DIG[MIN_Y] = 0;
						DIG[AM_D_X] = 0;DIG[AM_D_Y] = 0;
						DIG[AM_M_X] = 0;DIG[AM_M_Y] = 0;
						DIG[TEMP_X] = 0;DIG[TEMP_Y] = 0;
						data_time_display[DAY] = data_time[3];
			
						data_time_display[HOUR_X] = data_time[2] / 10;
						data_time_display[HOUR_Y] = data_time[2] % 10;
						data_time_display[MIN_X] = data_time[1] / 10;
						data_time_display[MIN_Y] = data_time[1] % 10;
						
						data_time_display[DATE_X] = data_time[4] / 10;
						data_time_display[DATE_Y] = data_time[4] % 10;
						
						data_time_display[MONTH_X] = data_time[5] / 10;
						data_time_display[MONTH_Y] = data_time[5] % 10;
						
						data_time_display[YEAR_X] = data_time[6] / 10;
						data_time_display[YEAR_Y] = data_time[6] % 10;
						
						lunar_convert();
						data_time_display[AM_D_X] = data_time[7] / 10;
						data_time_display[AM_D_Y] = data_time[7] % 10;
						
						data_time_display[AM_M_X] = data_time[8] / 10;
						data_time_display[AM_M_Y] = data_time[8] % 10;
						
						break;
				}
			}	else if(code_ir == KEY_INC){ 
				switch(mode_value){
					case 1:
						data_time[3]++;
						if(data_time[3] > 7) data_time[3] = 1;
						data_time_display[DAY] = bcd2dec(data_time[3]);
						break;
					case 2:
						data_time[4]++;
						if(data_time[4] > 31) data_time[4] = 1;
						data_time_display[DATE_X] = data_time[4] / 10;
						data_time_display[DATE_Y] = data_time[4] % 10;
						break;
					case 3:
						data_time[5]++;
						if(data_time[5] > 12) data_time[5] = 1;
						data_time_display[MONTH_X] = data_time[5] / 10;
						data_time_display[MONTH_Y] = data_time[5] % 10;
						break;
					case 4:
						data_time[6]++;
						if(data_time[6] > 99) data_time[6] = 0;
						data_time_display[YEAR_X] = data_time[6] / 10;
						data_time_display[YEAR_Y] = data_time[6] % 10;
						break;
					case 5:
						data_time[2]++;
						if(data_time[2] > 23) data_time[2] = 0;
						data_time_display[HOUR_X] = data_time[2] / 10;
						data_time_display[HOUR_Y] = data_time[2] % 10;
						break;
					case 6:
						data_time[1]++;
						if(data_time[1] > 59) data_time[1] = 0;
						data_time_display[MIN_X] = data_time[1] / 10;
						data_time_display[MIN_Y] = data_time[1] % 10;
						break;
				}
				reset_to_new_cmd();
			} else if(code_ir == KEY_DEC){ 
				switch(mode_value){
					case 1:
						data_time[3]--;
						if(data_time[3] < 1) data_time[3] = 7;
						data_time_display[DAY] = bcd2dec(data_time[3]);
						break;
					case 2:
						data_time[4]--;
						if(data_time[4] < 1) data_time[4] = 31;
						data_time_display[DATE_X] = data_time[4] / 10;
						data_time_display[DATE_Y] = data_time[4] % 10;
						break;
					case 3:
						data_time[5]--;
						if(data_time[5] < 1) data_time[5] = 12;
						data_time_display[MONTH_X] = data_time[5] / 10;
						data_time_display[MONTH_Y] = data_time[5] % 10;
						break;
					case 4:
						data_time[6]--;
						if(data_time[6] < 1) data_time[6] = 99;
						data_time_display[YEAR_X] = data_time[6] / 10;
						data_time_display[YEAR_Y] = data_time[6] % 10;
						break;
					case 5:
						data_time[2]--;
						if(data_time[2] > 200) data_time[2] = 23;
						data_time_display[HOUR_X] = data_time[2] / 10;
						data_time_display[HOUR_Y] = data_time[2] % 10;
						break;
					case 6:
						data_time[1]--;
						if(data_time[1] > 200) data_time[1] = 59;
						data_time_display[MIN_X] = data_time[1] / 10;
						data_time_display[MIN_Y] = data_time[1] % 10;
						break;
				}
				reset_to_new_cmd();
			}
			else {mode_value = 0; reset_to_new_cmd();}
		}
	}
}