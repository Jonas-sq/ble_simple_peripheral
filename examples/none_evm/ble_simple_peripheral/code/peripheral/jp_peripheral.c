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
    system_set_port_pull(GPIO_PB6, true);												/*PB3����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_6, PORTB6_FUNC_B6);						/*PB3����Ϊ PORTB3_FUNC_B3 ����*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_6, GPIO_DIR_OUT);								/*PB3����Ϊ ���ģʽ*/
    GPIO_SetBits(PORTB, STATUS_LED);
}

/*********************************************************************
 * @fn      JP_peripheral_init
 * @brief   before_ble_init �����н�����������ĳ�ʼ��
 * @param   None.
 * @return  None.
 */
void JP_peripheral_init(void)
{
    set_status_led();     // ����״̬��
    JP_UART0_init();
    JP_UART1_init();
    JP_GPIO_init();
}


/*********************************************************************
 * @fn      JP_GPIO_init
 * @brief   ʹ�õ�GPIO�ڽ��г�ʼ��
 * @param   None.
 * @return  None.
 */
void JP_GPIO_init(void)
{
    /* ACC����IO�� */
    system_set_port_pull(GPIO_PB3, true);												/*PB3����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_3, PORTB3_FUNC_B3);						/*PB3����Ϊ PORTB3_FUNC_B3 ����*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_3, GPIO_DIR_OUT);								/*PB3����Ϊ ���ģʽ*/

    /*  485_TXRX_EN  485ʹ�ܽų�ʼ��  */
    system_set_port_pull(GPIO_PB5, true);												/*PB5����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_5, PORTB5_FUNC_B5);						/*PB5����Ϊ PORTB5_FUNC_B5*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_5, GPIO_DIR_OUT);								/*PB5����Ϊ ���ģʽ*/
    GPIO_ResetBits(PORTB, RS485_EN);

    /* LED ���� */
    system_set_port_pull(GPIO_PB7, true);	                                            /*PB7����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_7, PORTB7_FUNC_B7);						/*PB7����Ϊ PORTB7_FUNC_B7*/
    gpio_set_dir(GPIO_PORT_B, GPIO_BIT_7, GPIO_DIR_OUT);								/*PB7����Ϊ ���ģʽ*/

    /* ����ͽ���źŵ͵�ƽ��� ����IO�ڸߵ�ƽ��Ч --��ʼ��ΪIO�ڵ͵�ƽ */
    system_set_port_pull(GPIO_PD0, true);												/*PD0����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_0, PORTB0_FUNC_B0);						/*PD0����Ϊ IO */
    gpio_set_dir(GPIO_PORT_D, GPIO_BIT_0, GPIO_DIR_OUT);								/*PD0����Ϊ ���ģʽ*/
    GPIO_ResetBits(PORTD, LOCK);
    system_set_port_pull(GPIO_PD0, true);												/*PD1����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_D, GPIO_BIT_0, PORTB0_FUNC_B0);						/*PD1����Ϊ IO */
    gpio_set_dir(GPIO_PORT_D, GPIO_BIT_0, GPIO_DIR_OUT);								/*PD1����Ϊ ���ģʽ*/
    GPIO_ResetBits(PORTD, UNLOCK);

    /* MK_MCU ��еԿ��/һ��������IO��  �����͵�ƽ��Ч ��Ȱ����� */
    pmu_set_pin_pull(GPIO_PORT_C, (1<<GPIO_BIT_2), true);							    /*PC2����Ϊ �͹�������ģʽ*/
    pmu_port_wakeup_func_set(GPIO_PC2);												    /*PC2       ʹ�ܵ͹��Ļ���*/
    button_init(GPIO_PC2);															    /*PC2 			������ʼ��*/

    /* LCOK_MCU ������ ��IO��  �����͵�ƽ��Ч ��Ȱ�����*/
    pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_2), true);							    /*PD2����Ϊ �͹�������ģʽ*/
    pmu_port_wakeup_func_set(GPIO_PD2);												    /*PD2       ʹ�ܵ͹��Ļ���*/
    button_init(GPIO_PD2);															    /*PD2 			������ʼ��*/

    //������ʼ�� PD3  ���񶯼�⣺�͵�ƽ��Ч��
    pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_3), true);							    /*PD3����Ϊ �͹�������ģʽ*/
    pmu_port_wakeup_func_set(GPIO_PD3);												    /*PD3       ʹ�ܵ͹��Ļ���*/
    button_init(GPIO_PD3);															    /*PD3 			������ʼ��*/
}

/*********************************************************************
 * @fn      JP_PWM_init()
 * @brief   PWM ���벶��
 * @param   None.
 * @return  None.
 */
void JP_PWM_init(void)
{

}

/*********************************************************************
 * @fn      JP_adc_init
 * @brief   ADC��⺯����ʼ��
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
 * @brief   485���ڳ�ʼ�� ������9600
 * @param   None.
 * @return  None.
 */
void JP_UART0_init(void)
{
    system_set_port_pull(GPIO_PB2, true);																		/*PB2����Ϊ ����ģʽ*/
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_2, PORTB2_FUNC_UART0_RXD);		/*PB2����Ϊ PORTB2_FUNC_UART0_RXD */
    system_set_port_mux(GPIO_PORT_B, GPIO_BIT_1, PORTB3_FUNC_UART0_TXD);		/*PB2����Ϊ PORTB3_FUNC_UART0_TXD */
    NVIC_EnableIRQ(UART0_IRQn);
    uart_init(UART0, BAUD_RATE_9600);
}

/*********************************************************************
 * @fn      JP_UART1_init
 * @brief   ���Դ��ڳ�ʼ�� ������115200
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
 * @brief   IO������Ϊ�ߵ�ƽ
 * @param   PORT - ��PORTA��PORTB��PORTC��PORTD��.
 *          PIN  - ��GPIO_Bit_x ��
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
 * @brief   IO������Ϊ�͵�ƽ
 * @param   PORT - ��PORTA��PORTB��PORTC��PORTD��.
 *          PIN  - ��GPIO_Bit_x ��
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

