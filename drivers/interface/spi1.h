/*
 * spi1.h
 *
 *  Created on: Nov 9, 2015
 *      Author: v
 */

#ifndef DRIVERS_INTERFACE_SPI1_H_
#define DRIVERS_INTERFACE_SPI1_H_
#include "stm32fxxx.h"
void spi1_init();

uint8_t spi1_send_byte(uint8_t dat);
uint8_t spi1_read_byte();

void readBytes(uint8_t addr, uint16_t index, uint8_t* buffer, uint8_t len);
void writeBytes(uint8_t addr, uint16_t index, uint8_t* buffer, uint8_t len);

int writetospi(uint8_t headerLen, uint8_t* headerBuff, uint8_t bodyLen, uint8_t* bodyBuff);
int readfromspi(uint8_t headerLen, uint8_t* headerBuff, uint8_t bodyLen, uint8_t* bodyBuff);

void spi1_device_enable();
void spi1_device_disable();

void spi1_changeRate(uint16_t scale);
int stop_irq();
void start_irq(int a);

#define SPIx	      SPI1
#define SPIx_GPIO     GPIOA
#define SPIx_SCK      GPIO_Pin_5
#define SPIx_SCK_SRC  GPIO_PinSource5
#define SPIx_MISO     GPIO_Pin_6
#define SPIx_MISO_SRC GPIO_PinSource6
#define SPIx_MOSI     GPIO_Pin_7
#define SPIx_MOSI_SRC GPIO_PinSource7
#define SPIx_CS       GPIO_Pin_8
#define SPIx_CSGPIO   GPIOB


#endif /* DRIVERS_INTERFACE_SPI1_H_ */
