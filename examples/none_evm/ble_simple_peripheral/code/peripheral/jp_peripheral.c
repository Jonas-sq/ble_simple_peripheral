#include "jp_peripheral.h"

os_timer_t adc_timer;


/*********************************************************************
 * @fn      status_led
 * @brief   MCU STATUS LED
 * @param   None.
 * @return  None.
 */
void set_status_led(void)
{
    system_set_port_pull(GPIO_PB6, true);												/*PB3设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_6, PORTB6_FUNC_B6);						/*PB3设置为 PORTB3_FUNC_B3 功能*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_6, GPIO_DIR_OUT);								/*PB3设置为 输出模式*/
    GPIO_SetBits(PORTB, STATUS_LED);
}

/*********************************************************************
 * @fn      JP_peripheral_init
 * @brief   before_ble_init 函数中进行所有外设的初始化
 * @param   None.
 * @return  None.
 */
void JP_peripheral_init(void)
{
    set_status_led();     // 点亮状态灯
    JP_UART0_init();
    JP_UART1_init();
    JP_GPIO_init();
}


/*********************************************************************
 * @fn      JP_GPIO_init
 * @brief   使用的GPIO口进行初始化
 * @param   None.
 * @return  None.
 */
void JP_GPIO_init(void)
{
    /* ACC控制IO口 */
    system_set_port_pull(GPIO_PB3, true);												/*PB3设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_3, PORTB3_FUNC_B3);						/*PB3设置为 PORTB3_FUNC_B3 功能*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_3, GPIO_DIR_OUT);								/*PB3设置为 输出模式*/

    /*  485_TXRX_EN  485使能脚初始化  */
    system_set_port_pull(GPIO_PB5, true);												/*PB5设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_5, PORTB5_FUNC_B5);						/*PB5设置为 PORTB5_FUNC_B5*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_5, GPIO_DIR_OUT);								/*PB5设置为 输出模式*/
    GPIO_ResetBits(PORTB, RS485_EN);

    /* LED 驱动 */
    system_set_port_pull(GPIO_PB7, true);	                                            /*PB7设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_7, PORTB7_FUNC_B7);						/*PB7设置为 PORTB7_FUNC_B7*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_7, GPIO_DIR_OUT);								/*PB7设置为 输出模式*/

    /* 设防和解防信号低电平输出 对于IO口高电平有效 --初始化为IO口低电平 */
    system_set_port_pull(GPIO_PD0, true);												/*PD0设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_0, PORTB0_FUNC_B0);						/*PD0设置为 IO */
    gpio_set_dir(GPIO_PORT_D, GPIO_BIT_0, GPIO_DIR_OUT);								/*PD0设置为 输出模式*/
    GPIO_ResetBits(PORTD, LOCK);
    system_set_port_pull(GPIO_PD0, true);												/*PD1设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_0, PORTB0_FUNC_B0);						/*PD1设置为 IO */
    gpio_set_dir(GPIO_PORT_D, GPIO_BIT_0, GPIO_DIR_OUT);								/*PD1设置为 输出模式*/
    GPIO_ResetBits(PORTD, UNLOCK);

    /* MK_MCU 机械钥匙/一键启动（IO口  读到低电平有效 类比按键） */
    pmu_set_pin_pull(GPIO_PORT_C, (1<<GPIO_BIT_2), true);							    /*PC2设置为 低功耗上拉模式*/
    pmu_port_wakeup_func_set(GPIO_PC2);												    /*PC2       使能低功耗唤醒*/
    button_init(GPIO_PC2);															    /*PC2 			按键初始化*/

    /* LCOK_MCU 坐垫检测 （IO口  读到低电平有效 类比按键）*/
    pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_2), true);							    /*PD2设置为 低功耗上拉模式*/
    pmu_port_wakeup_func_set(GPIO_PD2);												    /*PD2       使能低功耗唤醒*/
    button_init(GPIO_PD2);															    /*PD2 			按键初始化*/

    //按键初始化 PD3  （振动监测：低电平有效）
    pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_3), true);							    /*PD3设置为 低功耗上拉模式*/
    pmu_port_wakeup_func_set(GPIO_PD3);												    /*PD3       使能低功耗唤醒*/
    button_init(GPIO_PD3);															    /*PD3 			按键初始化*/
}

/*********************************************************************
 * @fn      JP_PWM_init()
 * @brief   PWM 输入捕获
 * @param   None.
 * @return  None.
 */
void JP_PWM_init(void)
{

}

/*********************************************************************
 * @fn      JP_adc_init
 * @brief   ADC检测函数初始化
 * @param   None.
 * @return  None.
 */
void JP_ACC_adc_init(void)
{
    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_4, PORTD4_FUNC_ADC0);
    os_timer_init(&adc_timer,adc_bp_tim_fn,NULL);
    os_timer_start(&adc_timer,1000,1);

    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_5, PORTD5_FUNC_ADC1);
    os_timer_init(&adc_timer,adc_acc_tim_fn,NULL);
    os_timer_start(&adc_timer,1000,1);

}
void adc_bp_tim_fn(void *arg)
{
    struct adc_cfg_t cfg;
    uint16_t result;
    memset((void*)&cfg, 0, sizeof(cfg));
    cfg.src = ADC_TRANS_SOURCE_PAD;
    cfg.ref_sel = ADC_REFERENCE_AVDD;
    cfg.channels = 0x01;
    cfg.route.pad_to_sample = 1;
    cfg.clk_sel = ADC_SAMPLE_CLK_24M_DIV13;
    cfg.clk_div = 0x3f;
    adc_init(&cfg);
    adc_enable(NULL, NULL, 0);
    adc_get_result(ADC_TRANS_SOURCE_PAD, 0x01, &result);
    adc_disable();
    co_printf("result:%d\r\n",result);
}
void adc_acc_tim_fn(void *arg)
{

}


/*********************************************************************
 * @fn      JP_UART0_init
 * @brief   485串口初始化 波特率9600
 * @param   None.
 * @return  None.
 */
void JP_UART0_init(void)
{
    system_set_port_pull(GPIO_PB2, true);																		/*PB2设置为 上拉模式*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_2, PORTB2_FUNC_UART0_RXD);		/*PB2设置为 PORTB2_FUNC_UART0_RXD */
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_1, PORTB3_FUNC_UART0_TXD);		/*PB2设置为 PORTB3_FUNC_UART0_TXD */
    NVIC_EnableIRQ(UART0_IRQn);
    uart_init(UART0, BAUD_RATE_9600);
}

/*********************************************************************
 * @fn      JP_UART1_init
 * @brief   调试串口初始化 波特率115200
 * @param   None.
 * @return  None.
 */
void JP_UART1_init(void)
{
    system_set_port_pull(GPIO_PA2, true);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_2, PORTA2_FUNC_UART1_RXD);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_3, PORTA3_FUNC_UART1_TXD);
    NVIC_EnableIRQ(UART1_IRQn);
	uart_init(UART1, BAUD_RATE_115200); 
}
/*********************************************************************
 * @fn      GPIO_SetBits
 * @brief   IO口设置为高电平
 * @param   PORT - （PORTA、PORTB、PORTC、PORTD）.
 *          PIN  - （GPIO_Bit_x ）
 * @return  None.
 */
void GPIO_SetBits(uint8_t PORT, uint8_t PIN)
{
    switch(PORT)
    {
        case PORTA:
        {
            gpio_porta_write(gpio_porta_read() | (1 << PIN));
            break;
        }
        case PORTB:
        {
            gpio_portb_write(gpio_portb_read() | (1 << PIN));
            break;
        }
        case PORTC:
        {
            gpio_portc_write(gpio_portc_read() | (1 << PIN));
            break;
        }
        case PORTD:
        {
            gpio_portd_write(gpio_portd_read() | (1 << PIN));
            break;
        }
    }
}
/*********************************************************************
 * @fn      GPIO_ResetBits
 * @brief   IO口设置为低电平
 * @param   PORT - （PORTA、PORTB、PORTC、PORTD）.
 *          PIN  - （GPIO_Bit_x ）
 * @return  None.
 */
void GPIO_ResetBits(uint8_t PORT, uint8_t PIN)
{
    switch(PORT)
    {
        case PORTA:
        {
            gpio_porta_write(gpio_porta_read() & ~(1 << PIN));
            break;
        }
        case PORTB:
        {
            gpio_portb_write(gpio_portb_read() & ~(1 << PIN));
            break;
        }
        case PORTC:
        {
            gpio_portc_write(gpio_portc_read() & ~(1 << PIN));
            break;
        }
        case PORTD:
        {
            gpio_portd_write(gpio_portd_read() & ~(1 << PIN));
            break;
        }
    }
}

