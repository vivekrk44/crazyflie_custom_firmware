/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart1.c - uart1 driver
 */
#include <string.h>
#include <math.h>

/*ST includes */
#include "stm32fxxx.h"
/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "nvicconf.h"
#include "config.h"
#include "uart1.h"
#include "cfassert.h"
#include "config.h"
#include "crtp.h"
#include "ledseq.h"
#include "console.h"
#include "debug.h"

#undef maxx
#define maxx(a,b) ((a) > (b) ? (a) : (b))
#undef minn
#define minn(a,b) ((a) < (b) ? (a) : (b))

#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CRTP_START_BYTE 0xAA

xSemaphoreHandle waitUntilSendDoneUART1 = NULL;
static xQueueHandle packet1Delivery;
static xQueueHandle uart1DataDelivery;
static uint8_t outBuffer[64];
static uint8_t dataIndex;
static uint8_t dataSize;
static uint8_t crcIndex = 0;
static bool    isUartDmaInitialized;
static bool isInit = false;
uint8_t state = 0;

uint8_t sbusBuffer[25];
uint8_t datBuffer[300];
static uint16_t sbusCh[20];

uint8_t failsafe;

struct sbus_dat {
	unsigned int ch0  : 11;
	unsigned int ch1  : 11;
	unsigned int ch2  : 11;
	unsigned int ch3  : 11;
	unsigned int ch4  : 11;
	unsigned int ch5  : 11;
	unsigned int ch6  : 11;
	unsigned int ch7  : 11;
	unsigned int ch8  : 11;
	unsigned int ch9  : 11;
	unsigned int ch10 : 11;
	unsigned int ch11 : 11;
	unsigned int ch12 : 11;
} __attribute__((__packed__));

typedef union {
	uint8_t sbusBuffer[25];
	struct sbus_dat chd;
} sbus_msg;

static sbus_msg sbus;

void uart1RxTask(void *param);
void uart1Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART1_GPIO_PERIF, ENABLE);
  ENABLE_UART1_RCC(UART1_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART1_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART1_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_GPIO_AF_TX_PIN, UART1_GPIO_AF_TX);
  GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_GPIO_AF_RX_PIN, UART1_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = UART1_BAUDRATE;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_2;
  USART_InitStructure.USART_Parity              = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART1_TYPE, &USART_InitStructure);
  USART_Cmd(UART1_TYPE, ENABLE);
  USART_ITConfig(UART1_TYPE, USART_IT_RXNE, ENABLE);
  //Enable NVIC
  NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  xTaskCreate(uart1RxTask, (const signed char * const)"UART-Rx", 1024, NULL, /*priority*/2, NULL);
  packet1Delivery = xQueueCreate(2, sizeof(CRTPPacket));
  uart1DataDelivery = xQueueCreate(40, sizeof(uint8_t));

  //Enable UART

  isInit = true;
}

bool uart1Test(void)
{
  //uart1Printf("Hello UART1!\n");
  return isInit;
}

void uart1RxTask(void *param)
{
  enum {waitForFirstStart, waitForData, waitForFlags, waitForEnd} rxstate;
  uint8_t c;
  uint8_t startByte;
  uint8_t crc;
  CRTPPacket p;
  rxstate = waitForFirstStart;
  uint8_t counter =0;
  uint8_t datCounter = 0;
  uint8_t fc = 0;
  uint8_t stb = 0x0F;
  uint8_t enb = 0x00;
  uint8_t flag = 0x00;
  while(1)
  {
    if(xQueueReceive(uart1DataDelivery, &c, UART_DATA_TIMEOUT_TICKS)==pdTRUE)
    {
      counter++;
      //c = ~c;
      //c = convertLE(c);
      datBuffer[datCounter++] = c;
      //if(c==stb)
    	  //rxstate = waitForFirstStart;
      switch(rxstate)
      {
        case waitForFirstStart:
          rxstate = (c == stb) ? waitForData : waitForFirstStart;
          startByte = c;
          dataIndex = 0;
          break;
        case waitForData:
          sbus.sbusBuffer[dataIndex] = c;
          crc = 0; //(crc +c)%0xFF
          dataIndex++;
          if(dataIndex >= 22)
            rxstate = waitForFlags;
          break;

        case waitForFlags:
          flag = c;
          rxstate = waitForEnd;

          break;
        case waitForEnd:
          if(c!= enb)
          {
            rxstate = waitForFirstStart;
            fc += 1;
          }
          else
          {
				  sbusCh[0] = sbus.chd.ch0;//sbus.chd.ch0;//((sbusBuffer[0]    | sbusBuffer[1]<<8)                     & 0x07FF);
				  sbusCh[1] = sbus.chd.ch1;//sbus.chd.ch1;//((sbusBuffer[1]>>3 | sbusBuffer[2]<<5)                     & 0x07FF);
				  sbusCh[2] = sbus.chd.ch2;//sbus.chd.ch2;//((sbusBuffer[2]>>6 | sbusBuffer[3]<<2 | sbusBuffer[4]<<10) & 0x07FF);
				  sbusCh[3] = sbus.chd.ch3;//sbus.chd.ch3;//((sbusBuffer[4]>>1 | sbusBuffer[5]<<7)                     & 0x07FF);
				  sbusCh[4] = sbus.chd.ch4;//((sbusBuffer[5]>>4 | sbusBuffer[6]<<4)                     & 0x07FF);
				  sbusCh[5] = sbus.chd.ch5;
				  sbusCh[6] = sbus.chd.ch6;
				  sbusCh[7] = sbus.chd.ch7;
				  sbusCh[8] = sbus.chd.ch8;
				  sbusCh[9] = sbus.chd.ch9;
				  sbusCh[10] = sbus.chd.ch10;
				  sbusCh[11] = sbus.chd.ch11;
				  sbusCh[12] = sbus.chd.ch12;

				  roll  = ((sbusCh[0] - 1024)/336.0f)*SBMRP;
				  pitch = ((sbusCh[1] - 1024)/336.0f)*SBMRP;

				  thrust = ((sbusCh[2] - 352)/1344.0f)*SBMT;
				  yaw    = ((sbusCh[3] - 1024)/672.0f)*SBMY;

				  roll =   maxx(minn(roll, SBMRP), -SBMRP);
				  pitch =  maxx(minn(pitch, SBMRP), -SBMRP);

				  thrust = maxx(minn(thrust, SBMT), -SBMT);
				  yaw = maxx(minn(yaw, SBMY), -SBMY);
				  failsafe = 0x00;
				  failsafe = flag & 0x04;

				  updated  = 1;

				  datCounter = 0;
				  rxstate = waitForFirstStart;
				  fc = 0;
          }
          break;
        default:
          ASSERT(0);
          break;
      }
    }
    else
    {
      //Timeout
      rxstate = waitForFirstStart;
    }
  }
}

int getSBusUpdate()
{
	return updated;
}

void setSBusUpdate(int val)
{
	updated = val;
}

void getSBusVals(float* r, float* p, float* y, uint16_t* t)
{
	*t = thrust;
	*p = pitch;
	*r = roll;
	*y = yaw;
}

uint16_t getSBusChannel(uint8_t chno)
{
	if(failsafe)
		return 0;
	else
		return sbusCh[chno-1];
}

static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint8_t rxDataInterrupt;
void uart1Issre(void)
{
  USART_ClearITPendingBit(UART1_TYPE, USART_IT_TXE);

}

void uart1SendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
    while (!(UART1_TYPE->SR & USART_FLAG_TXE));
    UART1_TYPE->DR = (data[i] & 0x00FF);
  }
}

int uart1Putchar(int ch)
{
  uart1SendData(1, (uint8_t *)&ch);

  return (unsigned char)ch;
}
void __attribute__((used)) UART4_IRQHandler(void)
{
  uart1Issre();
  xHigherPriorityTaskWoken = pdFALSE;
  if(USART_GetITStatus(UART1_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UART1_TYPE) & 0xFF;
    xQueueSendFromISR(uart1DataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }

}


uint8_t convertLE(uint8_t c)
{
	uint8_t a = ((c&0b00000001)<<7) | ((c&0b00000010)<<5) | ((c&0b00000100)<<3) | ((c&0b00001000)<<1) | ((c&0b00010000)>>1) | ((c&0b00100000)>>3) | ((c&0b0100000)>>5) | ((c&0b1000000)>>7);
	return a;
}
