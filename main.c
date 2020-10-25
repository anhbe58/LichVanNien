/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"

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
uint8_t seg = 0, value = 99, n = 0;
uint8_t data_display[17] = {1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1, 2, 3};
//IR
uint8_t done = 0, is_repeat = 0;
int16_t count_ms1 = 0, count_bit_ir = 0, start_status = -1;
uint32_t code_ir = 0;
//IR end init

//I2C
#define I2C_READ	1
#define I2C_WRITE	0
uint8_t reg = 0;
uint8_t year_disp = 0;
//I2C end init
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
			GPIOB->ODR  =    led7_b[data_display[DATE_X]];

			GPIOD->ODR  =    led7_d[data_display[MIN_X]];
			
			GPIOA->ODR = 0b01000100;
			
			GPIOC->ODR = 0b00000000;
		
			GPIOE->ODR = 0b00001000;
			GPIOG->ODR = 0b00000000;
			break;
		}					
		case 2:
		{
			GPIOB->ODR  =    led7_b[data_display[DATE_Y]];

			GPIOD->ODR  =    led7_d[data_display[MIN_Y]];
			
			GPIOA->ODR = 0b00100100;
			
			GPIOC->ODR = 0b00000000;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000010;
			break;
		}						
		case 3:
		{
			GPIOB->ODR  =    led7_b[data_display[MONTH_X]];

			GPIOD->ODR  =    led7_d_1[data_display[AM_D_X]];
						
			GPIOA->ODR = 0b00000110;
			
			GPIOC->ODR = 0b00000000;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000001;
			break;
		}
		case 4:
		{
			GPIOB->ODR  =    led7_b[data_display[MONTH_Y]];

			GPIOD->ODR  =    led7_d_1[data_display[AM_D_Y]];
						
			GPIOA->ODR = 0b00000100;
			
			GPIOC->ODR = 0b01100000;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 5:
		{
			GPIOB->ODR  =    led7_b[data_display[YEAR_X]];

			GPIOD->ODR  =    led7_d_1[data_display[AM_M_X]];			
			GPIOA->ODR = 0b00000100;
			
			GPIOC->ODR = 0b00001010;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 6:
		{
			GPIOB->ODR  =    led7_b[data_display[YEAR_Y]];

			GPIOD->ODR  =    led7_d_1[data_display[AM_M_Y]];			
						
			GPIOA->ODR = 0b00001100;
			
			GPIOC->ODR = 0b00000000;
		
			GPIOE->ODR = 0b00100000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 7:
		{
			GPIOB->ODR  =    led7_b[data_display[HOUR_X]];//data_display[HOUR_X]

			GPIOD->ODR  =    led7_d_1[data_display[TEMP_X]];			
						
			GPIOA->ODR = 0b00010100;
			
			GPIOC->ODR = 0b00010000;
		
			GPIOE->ODR = 0b00000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 8:
		{
			GPIOB->ODR  =    led7_b[data_display[DAY]];

			GPIOD->ODR  =    led7_d[data_display[HOUR_Y]];	
						
			GPIOA->ODR = 0b00000100;
			
			GPIOC->ODR = 0b10000000;
		
			GPIOE->ODR = 0b10000000;
			GPIOG->ODR = 0b00000000;
			break;
		}
		case 9:
		{
			GPIOD->ODR  =    led7_d_1[data_display[TEMP_Y]];
						
			GPIOA->ODR = 0b00000000;
			
			GPIOC->ODR = 0b00000100;
		
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
	if(seg > 9)
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
	
	if((start_status == 0) || (done == 1)) reset_to_new_cmd();
	//mode = 0;
	//clear trigger
	//data_display[1] = 0; data_display[2] = 0;
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
				} else if(count_ms1 <= 180)//10ms-14ms start, repeat 9ms+ 2.25ms = 11.25
				{
					/*
					count_bit_ir=0;
					start_status=-1;
					count_ms1=0;
					done = 1;
					data = 0xFFFFFFFF;
					*/
					TIM2->CNTRL = 0x16;
					TIM2->CNTRH = 0xF7;	
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
					data_display[15] = ((uint8_t) count_bit_ir) / 10;
					data_display[16] = ((uint8_t) count_bit_ir)  % 10;	
					TIM2->CNTRL = 0xCB;
					TIM2->CNTRH = 0xF3;	
					is_repeat = 0;
					done = 1;
					
				}
				break;
		}
	}
}
void timer4_init(void) {
	// CK_PSC (internal fMASTER) is divided by the prescaler value.
	TIM4->PSCR = 7;
	TIM4->ARR = 131;
	// Enable update interrupt for timer 4
	TIM4->IER |= (1 << 0);
	// Clear timer interrupt flag
	TIM4->SR1 &= ~(1 << 0);
	// Precalculated value
	//TIM4_CNTR = 0xFF - 126;
	// Enable timer 4
	TIM4->CR1 |= (1 << 0);
	//Enable auto-reload
	TIM4->CR1 |= (1 << 7);
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
void test_i2c(void){
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
	
	I2C->DR = 0x06;
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);
	
	//generate STOP
	I2C->CR2 = 0b00000110;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	
	I2C->CR2 = 0b00000101;
	//wait START generate complete
	while (((I2C->CR2) & 0x01) == 0x01);
	//clear SB bit after START complete
	while(((I2C->SR1) & 0x01) == 0x00);
	//write data to bus
	I2C->DR = (0x68 << 1) | I2C_READ;
	
	//wait ADDR bit is set
	while (((I2C->SR1 >> 1) & 0x01) == 0x00);
	//set NACK after receive one byte
	I2C->CR2 = 0b00000000;
	reg = I2C->SR1;
	reg = I2C->SR3;
	

	//wait ADDR bit is cleared
	while (((I2C->SR1 >> 1) & 0x01) == 0x01);
	//set STOP after receive done
	I2C->CR2 = 0b00000010;
	year_disp = I2C->DR;
	reg = I2C->SR3;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	
}
uint8_t bcd2dec(uint8_t num)
{
	return ((num/16 * 10) + (num % 16));
}
uint8_t dec2bcd(uint8_t num)
{
	return ((num/10 * 16) + (num % 10));
}
void setTime(uint8_t hr, uint8_t min, uint8_t sec, uint8_t wd, uint8_t d, uint8_t mth, uint8_t yr)
{
	//  Enter master mode.
	I2C->CR2 = 0b00000101;
	//wait START generate complete
	while (((I2C->CR2) & 0x01) == 0x01);
	//clear SB bit after START complete
	while(((I2C->SR1) & 0x01) == 0x00);
	//write data to bus
	I2C->DR = (0x68 << 1) | 0;
	//wait ADDR bit is set
	while (((I2C->SR1 >> 1) & 0x01) == 0x00);
	reg = I2C->SR1;
	reg = I2C->SR3;
	reg = I2C->SR3;
	//end address phase
	
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
	
	I2C->DR = dec2bcd(yr);
	//wait TXE bit is set
	while (((I2C->SR1 >> 7) & 0x01) == 0x00);

	//generate STOP and wait
	I2C->CR2 = 0b00000110;
	while (((I2C->CR2 >> 1) & 0x01) == 0x01);
	
}
main()
{
	CLK->CKDIVR = 0x00; // Set the frequency to 16 MHz
	delay(1000);
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
	GPIOE->CR2 = 0b11101001;
	GPIOG->CR2 = 0b00000011;
	timer4_init();
	timer2_init();
	i2c_init();
	EXTI->CR2 = 0b00000010;
	//ITC->ISPR2 = 0b00111111;
	//ITC->ISPR6 = 0b01111111;
	enableInterrupts();
	//setTime(19, 30, 0, 2, 12, 10, 29);
	test_i2c();
	while (1){
		//GPIOA->ODR = 0b01000100;//
		//GPIOB->ODR = led7_b[8];// led data left from second
		//GPIOC->ODR = 0b00000000;//
		//GPIOD->ODR = led7_d[8];// led data right from second
		//GPIOE->ODR = 0b00001000;//
		//GPIOG->ODR = 0b00000000;//
		
		if(done == 1){
			if(code_ir == 0x40BDA25D){ data_display[10] = 9; reset_to_new_cmd();}
			/*
			else if(data == 0xFFFFFFFF){
					mode = 1; delay(50000); mode = 0; delay(50000);
					reset_to_new_cmd();
			}
			*/
			else {data_display[10] = 0; reset_to_new_cmd();}
		}
	}
}