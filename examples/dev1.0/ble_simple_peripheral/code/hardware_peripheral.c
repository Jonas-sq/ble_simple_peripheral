#include <stdbool.h>
#include "gap_api.h"
#include "gatt_api.h"
#include "driver_gpio.h"
#include "driver_pmu.h"
#include "button.h"
#include "os_timer.h"
#include "speaker_service.h"
#include "simple_gatt_service.h"
#include "ble_simple_peripheral.h"

#include "hardware_peripheral.h"

void hardware_led_init(void);
void hardware_gpio_init(void);


void hardware_peripheral_init(void)
{
	hardware_gpio_init();
	//hardware_led_init();
}

void hardware_led_init(void)
{
	pmu_set_led2_value(true);
}

void hardware_gpio_init(void)
{
	/*普通IO口初始化  PA1*/
	system_set_port_mux(GPIO_PORT_A, GPIO_BIT_1, PORTA1_FUNC_A1);
	gpio_set_dir(GPIO_PORT_A, GPIO_BIT_1, GPIO_DIR_OUT);
	gpio_porta_write(gpio_porta_read() | GPIO_PIN_1);
}
