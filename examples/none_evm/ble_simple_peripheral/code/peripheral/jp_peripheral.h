#ifndef _JP_PERIPHERAL_H_
#define _JP_PERIPHERAL_H_

#include <stdio.h>
#include <string.h>
#include "driver_gpio.h"
#include "driver_iomux.h"
#include "driver_uart.h"
#include "driver_adc.h"
#include "driver_pmu.h"

#include "co_printf.h"
#include "os_timer.h"
#include "button.h"

#define  RS485_EN     GPIO_BIT_5   
#define  LOCK         GPIO_BIT_0
#define  UNLOCK       GPIO_BIT_1
#define  STATUS_LED   GPIO_BIT_6

enum jp_port
{
    PORTA,
    PORTB,
    PORTC,
    PORTD
};


void set_status_led(void);
void JP_GPIO_init(void);
void JP_peripheral_init(void);
void JP_UART0_init(void);
void JP_UART1_init(void);
void GPIO_SetBits(uint8_t PORT, uint8_t PIN);
void GPIO_ResetBits(uint8_t PORT, uint8_t PIN);

void JP_adc_init(void);
void adc_bp_tim_fn(void *arg);
void adc_acc_tim_fn(void *arg);
#endif
