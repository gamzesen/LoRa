#include <stdio.h>
#include "stm32f407xx.h"

/************************************
************  functions  ************
************************************/
void GPIO_Config();
void USART_Config();
void Sleep_Config();
void USART2_IRQHandler();
void USART_tx (char b);
uint8_t sensor_get_value();
/*************************************
*************  variables  ************
*************************************/
const uint8_t destination_address = 0x0014;
const uint8_t source_address = 0x0014;
char destination_incoming;
char source_incoming;
char framebuffer[7];
uint8_t adc_buffer[9] = {14, 12, 18, 14, 3, 97, 23, 10, 96};
char rx_buffer[7] = {0};
int frame_ready = 0;

// RTC - timestamp için

// timer interupt. ADC datası sample simulasyon
// adc_buffer next elemanını gönderecek

// 1. uart data gönder frame hazırlayıp
// 2. uart dan data al kendi destination için
// 3. uart dan data al başka destination için (forwarding logic _ tabling, hop count)
// 4. timestamp implement - checks
// 5. checksum implement - checks
// 6. initalize lora fonksyonu (registerlarını set edecek)

struct frame_t
{
	uint8_t dest;
	uint8_t source;
	uint8_t hop;
	//uint32_t timestamp;  // zaman nasil tutuluyor
	uint8_t msg[4];
	//uint8_t checksum;
};
void send_frame(struct frame_t *f);


int main()
{
	GPIO_Config();
	USART_Config();
	//Sleep_Config();

	struct frame_t f;
	f.dest = destination_address;
	f.source = source_address;
	f.hop = 0;


	send_frame(&f);

	while(1)
	{
		f.msg[0] = sensor_get_value();
	}
}

void GPIO_Config(void)
{
	RCC->AHB1ENR |= 0x00000001;    // Set Bit 0 to enable GPIOA clock in AHB1ENR
	RCC->AHB1ENR |= 0x00000008;    // Set Bit 3 to enable GPIOD clock in AHB1ENR

    GPIOA->MODER &= 0xFFFFFFFC;   // Reset bits 0-1 to clear old values
    GPIOA->MODER |= 0x00000000;   // Make button an input

	GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
	GPIOD->MODER |= 0x55000000;   // Set MODER bits to 01 (0101 is 5 in hex)
}

void USART_Config(void)
{
    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    GPIOA->MODER &= 0xFFFFFF0F; // Reset bits 10-15 to clear old values
    GPIOA->MODER |= 0x000000A0; // Set pin 2/3 to alternate func. mode (0b10)

    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin 2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin 3

    // usart2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // usart2 parity control, bit 9
    //USART2->CR1 |= (0 << 9); // 0 - no parity

    // usart2 tx enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // usart2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);

    USART2->CR1 |= (1 << 5);

    // Set USART2 NVIC Priority To 1
    NVIC_SetPriority(USART2_IRQn,0);
    // Enable USART2 NVIC
    NVIC_EnableIRQ(USART2_IRQn);

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 42 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 42Mhz / 115200 / 16
    //   = 22.7864 22.8125
    // we can also look at the table in RM0090
    //   for 42 Mhz PCLK, OVER8 = 0 and 115.2 KBps baud
    //   we need to program 22.8125
    // Fraction : 16*0.8125 = 13 (multiply fraction with 16)
    // Mantissa : 22
    // 12-bit mantissa and 4-bit fraction
    USART2->BRR |= (22 << 4);
    USART2->BRR |= 13;

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);
}

void USART2_IRQHandler(void)
{
    if (((USART2->SR) >> 5) & 0x01)    // If Is RXNE Enable
    {
        static int i = 0;
        rx_buffer[i++] = USART2->DR;
        if (i == 58)
		{
        	frame_ready = 1;
        	i = 0;
		}

        else
        {
            USART2->SR &= 0xFFFFFFDF;    // Clear RXNE Bit In USART2 Status Register
        }
    }
}
void USART_tx (char b)
{
	 USART2->DR = b;
	 // wait for transmit complete
	 while(!(USART2->SR & (1 << 6)));  // Check Wheter The Transmittion Is Complete Or Not
}


/*
void check �f �n buffer ()
{
	// msg bizim icin gelmis - ona gore process et
	// msg l�stede var m�  varsa �gnore
	// msg l�stede yoksa  l�steye ekle
	
	for � �n msgbuffer struc
	 �f �.source  == rxbuf 0 
	    
	 
}

*/
void send_frame(struct frame_t *f)
{
	framebuffer[0] = f->dest;
	framebuffer[1] = f->source;
	framebuffer[2] = f->hop;
	
    for (uint32_t i=3; i<=6; i++)
    {
		framebuffer[i] = f->msg[i-3];
	}
    for (uint32_t i=0; i<=58; i++)
    {
        // send character +
    	USART_tx(framebuffer[i]);
    }

}

/*
void Sleep_Config(void)
{
	// Note about DeepSleep mode, the oscillator will be selected as the internal one
	// (HSI) upon waking up
    SCB->SCR |= (1 << 1); // SleepOnExit

	// sleep
    __WFI(); // wait for interrupt
}
*/
/*
void comparedestination(void)
{
	if (0 == strncmp (destination_incoming, source) )
	{
		// dosya işlemleri...
	}
	else
	{
		// go to
	}
}
*/

uint8_t sensor_get_value(void)
{
	static uint32_t i = 0;
	uint8_t value = 0;

	value = adc_buffer[i];
	i++;

	if(i == 9)
	{
		i = 0;
	}

	return value;
}
