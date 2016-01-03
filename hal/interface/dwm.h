/*
 * dwm.h
 *
 *  Created on: Nov 9, 2015
 *      Author: v
 */

#ifndef HAL_INTERFACE_DWM_H_
#define HAL_INTERFACE_DWM_H_
#include "stm32fxxx.h"
#include "deca_device_api.h"

void dwm_init();
void setup_irq();
void setup_reset();
void reset_DW1000();

void dwmTask(void*);

void int_tx(const dwt_callback_data_t *rxd);
void int_rx(const dwt_callback_data_t *txd);

#define DWMIRQ_GPIO  GPIOC
#define DWMEXTI_SRC  EXTI_PortSourceGPIOC
#define DWMIRQ_PIN   GPIO_Pin_12
#define DWMEXTI_PIN  EXTI_PinSource12
#define DWMEXTI_LINE EXTI_Line12
#define DWMNVIC_IRQ  EXTI15_10_IRQn

#define DWMRST_PIN  GPIO_Pin_6
#define DWMRST_GPIO GPIOB

dwt_config_t dwm_conf;
dwt_txconfig_t dwm_txconf;

#define channelNo   5
#define preambleCo  0x04
#define pulseRF     DWT_PRF_16M
#define preambleLen DWT_PLEN_1024
#define dataR       DWT_BR_110K
#define recievePAC  DWT_PAC32
#define nonSFD      0
#define SFDto       (1025 + 64 - 32)
#define phrMo       0x0



#endif /* HAL_INTERFACE_DWM_H_ */
