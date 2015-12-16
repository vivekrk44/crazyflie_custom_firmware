/*
 * spi1.c
 *
 *  Created on: Nov 9, 2015
 *      Author: v
 */

//ToDo - Clean this
#include "spi1.h"
#include "stm32fxxx.h"
#include "dwm.h"


void spi1_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = SPIx_SCK | SPIx_MOSI | SPIx_MISO;
	GPIO_Init(SPIx_GPIO, &GPIO_InitStruct);

	GPIO_PinAFConfig(SPIx_GPIO, SPIx_SCK_SRC, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPIx_GPIO, SPIx_MOSI_SRC, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPIx_GPIO, SPIx_MISO_SRC, GPIO_AF_SPI1);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = SPIx_CS;
	GPIO_Init(SPIx_CSGPIO, &GPIO_InitStruct);

	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CRCPolynomial = 0;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_Init(SPI1, &SPI_InitStruct);

	spi1_device_disable();

	SPI_Cmd(SPI1, ENABLE);
}

void spi1_device_disable()
{
	GPIO_SetBits(SPIx_CSGPIO, SPIx_CS);
}

void spi1_device_enable()
{
	GPIO_ResetBits(SPIx_CSGPIO, SPIx_CS);
}

uint8_t spi1_send_byte(uint8_t dat)
{
	//spi1_device_enable();
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI1, dat);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	return SPI_I2S_ReceiveData(SPI1);
}

uint8_t spi1_read_byte()
{
	return spi1_send_byte(0xff);
}

int writetospi(uint8_t headerLen, uint8_t* headerBuff, uint8_t bodyLen, uint8_t* bodyBuff)
{
	int irq_stat = stop_irq();
	spi1_device_enable();
	int i = 0;
	for(i=0 ; i< headerLen ; i++)
	{
		spi1_send_byte(headerBuff[i]);
	}

	for(i=0 ; i<bodyLen ; i++)
	{
		spi1_send_byte(bodyBuff[i]);
	}
	spi1_device_disable();
	start_irq(irq_stat);
	return 0;
}
void writeBytes(uint8_t addr, uint16_t index, uint8_t* buffer, uint8_t len)
{
	spi1_device_enable();
	addr = 0x80 | addr; // sets 8th bit to 1 for writing

	if (index == 0)
		spi1_send_byte(addr);
	else if (index > 0 && index < 127)
	{
		addr = 0x40 | addr; //sets 7th bit to 1 for indicating the presence of index
		spi1_send_byte(addr);
		spi1_send_byte(index & 0x7F); //clears 8th bit to remove 3rd address octet
	}
	else if(index >= 127)
	{
		//Not Supported
		return;
	}
	int i;
	for (i=0 ; i<len ; i++)
		spi1_send_byte(buffer[i]);
	spi1_device_disable();
}

int readfromspi(uint8_t headerLen, uint8_t* headerBuff, uint8_t bodyLen, uint8_t* bodyBuff)
{
	int irq_stat = stop_irq();
	spi1_device_enable();

	int i=0;

	for(i=0 ; i<headerLen ; i++)
	{
		spi1_send_byte(headerBuff[i]);
	}

	for(i=0 ; i<bodyLen ; i++)
	{
		bodyBuff[i] = spi1_read_byte();
	}
	spi1_device_disable();
	start_irq(irq_stat);
	return 0;
}

void readBytes(uint8_t addr, uint16_t index, uint8_t* buffer, uint8_t len)
{
	spi1_device_enable();
	addr = 0x7F & addr; //clears 8th bit to 0 for reading
	if (index == 0)
		spi1_send_byte(addr);
	else if (index > 0 && index < 127)
	{
		addr = 0x40 | addr; //sets 7th bit to 1 for indicating the presence of index
		spi1_send_byte(addr);
		spi1_send_byte(index & 0x7f); //clears 8th bit to remove 3rd address octet
	}
	else if (index > 127)
	{
		//Not Supported
		return;
	}
	int i;
	for (i=0 ; i<len ; i++)
		buffer[i] = spi1_read_byte();
}

void spi1_changeRate(uint16_t scale)
{
	uint16_t temp = 0;
	temp = SPIx->CR1;
	temp &= 0xFFC7;
	temp |= scale;
	SPIx->CR1 = temp;
}

int stop_irq()
{
	int s = EXTI_GetITStatus(DWMNVIC_IRQ);
	if (s)
		NVIC_DisableIRQ(DWMNVIC_IRQ);
	return s;
}

void start_irq(int a)
{
	if (a)
		NVIC_EnableIRQ(DWMNVIC_IRQ);
}
