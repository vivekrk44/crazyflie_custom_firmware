/*
 * dwm.c
 *
 *  Created on: Nov 9, 2015
 *      Author: v
 */
#include "dwm.h"
#include "stm32fxxx.h"
#include "spi1.h"
#include "debug.h"
#include "deca_device_api.h"



void dwm_init()
{
    spi1_init();
    setup_irq();
    setup_reset();

    uint32_t dev_id = dwt_readdevid();
    if(dev_id != 0xDECA0130)
    	//while (1); // Error

    dwt_setinterrupt(DWT_INT_RFCG, 1);
    dwt_setcallbacks(int_tx, int_rx);

    int use_otpdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;

    dwt_setdblrxbuffmode (0);
    dwt_enableframefilter(DWT_FF_COORD_EN);
    dwt_setpanid(0x000A);
    dwt_setaddress16(0x0009);

    //dwt_initialise(use_otpdata);

    dwm_conf.chan           = channelNo;
    dwm_conf.rxCode         = preambleCo ;
    dwm_conf.txCode         = preambleCo;
    dwm_conf.prf            = pulseRF;
    dwm_conf.dataRate       = dataR ;
    dwm_conf.txPreambLength = preambleLen;
    dwm_conf.rxPAC          = recievePAC;
    dwm_conf.nsSFD          = nonSFD;
    dwm_conf.phrMode        = phrMo;
    dwm_conf.sfdTO          = SFDto;
    dwt_configure(&dwm_conf, use_otpdata);


    dwt_setrxtimeout(0);
    int rx_stat = dwt_rxenable(0);
    if(rx_stat)
    	while(1);
    
}

void setup_reset()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = DWMRST_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(DWMRST_GPIO, &GPIO_InitStruct);
	GPIO_SetBits(DWMRST_GPIO, DWMRST_PIN);
}

void setup_irq()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = DWMIRQ_PIN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(DWMIRQ_GPIO, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(DWMEXTI_SRC, DWMEXTI_PIN);

    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Line = DWMEXTI_LINE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannel = DWMNVIC_IRQ;
    NVIC_Init(&NVIC_InitStruct);
}

void reset_DW1000()
{
	GPIO_ResetBits(DWMRST_GPIO, DWMRST_PIN);
	vTaskDelay(1);
	GPIO_SetBits(DWMRST_GPIO, DWMRST_PIN);
}

void int_rx(const dwt_callback_data_t *rxd)
{
	uint32_t stat;
	uint8_t event;
	uint8_t aack;
	uint8_t dblbf;

	stat = rxd->status;
	event = rxd->event;
	aack = rxd->aatset;
	dblbf = rxd->dblbuff;

	//Dont know what to do :(
}

void int_tx(const dwt_callback_data_t *txd)
{
	uint32_t stat;
	uint8_t event;
	uint8_t aack;
	uint8_t dblbf;

	stat = txd->status;
	event = txd->event;
	aack = txd->aatset;
	dblbf = txd->dblbuff;
	if (event == DWT_SIG_RX_OKAY)
		while (1);
	//Nothing to do here
}
