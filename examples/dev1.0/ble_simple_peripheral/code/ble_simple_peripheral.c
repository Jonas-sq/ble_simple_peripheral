/**
 * Copyright (c) 2019, Freqchip
 * 
 * All rights reserved.
 * 
 * 
 */
 
/*
 * INCLUDES (包含头文件)
 */
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

#include "sys_utils.h"
#include "lcd.h"
#include "capb18-001.h"
#include "sht3x.h"
#include "decoder.h"
#include "gyro_alg.h"
#include "flash_usage_config.h"
#include "hardware_peripheral.h"



/*
 * MACROS
 */

/*
 * CONSTANTS 
 */
const unsigned	char * lcd_show_workmode[MODE_MAX] = {"PICTURE_UPDATE","SENSOR_DATA","SPEAKER_FROM_FLASH","CODEC_TEST"};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
// GAP-广播包的内容,最长31个字节.短一点的内容可以节省广播时的系统功耗.
static uint8_t adv_data[] =
{
  // service UUID, to notify central devices what services are included
  // in this peripheral. 告诉central本机有什么服务, 但这里先只放一个主要的.
  0x03,   // length of this data
  GAP_ADVTYPE_16BIT_MORE,      // some of the UUID's, but not all
  0xFF,
  0xFE,
};

// GAP - Scan response data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
// GAP-Scan response内容,最长31个字节.短一点的内容可以节省广播时的系统功耗.
static uint8_t scan_rsp_data[] =
{
  // complete name 设备名字
  0x12,   // length of this data
  GAP_ADVTYPE_LOCAL_NAME_COMPLETE,
  'S','i','m','p','l','e',' ','P','e','r','i','p','h','e','r','a','l',

  // Tx power level 发射功率
  0x02,   // length of this data
  GAP_ADVTYPE_POWER_LEVEL,
  0,	   // 0dBm
};

/*
 * TYPEDEFS 
 */

/*
 * GLOBAL VARIABLES 
 */

os_timer_t timer_refresh;// 用于刷新传感器数据以及显示等
os_timer_t timer_count;// 计数用定时器
uint8_t App_Mode = PICTURE_UPDATE;//工作模式  可以通过KEY1切换

/*
 * LOCAL VARIABLES 
 */
 
/*
 * LOCAL FUNCTIONS
 */

static void sp_start_adv(void);
/* =====================  广播函数  =============================   */

/*
 * EXTERN FUNCTIONS
 */
uint8_t CAPB18_data_get(float *temperature,float *air_press);
uint8_t demo_CAPB18_APP(void);

/*
 * PUBLIC FUNCTIONS
 */

/** @function group ble peripheral device APIs (ble外设相关的API)
 * @{
 */




uint8_t sp_conidx;//蓝牙连接号conidx
extern uint8_t sp_svc_id;//服务号
/*********************************************************************
 * @fn      app_gap_evt_cb
 *
 * @brief   Application layer GAP event callback function. Handles GAP evnets.
 *					应用层GAP事件回调函数，处理GAP事件
 * @param   p_event - GAP events from BLE stack.
 *       		蓝牙协议栈产生的 GAP事件
 *
 * @return  None.
 */
void app_gap_evt_cb(gap_event_t *p_event)
{
    switch(p_event->type)
    {
        case GAP_EVT_ADV_END:																													//广播结束。示例：adv_end,status:0x00
        {
            co_printf("adv_end,status:0x%02x\r\n",p_event->param.adv_end.status);	
            //gap_start_advertising(0);
        }
        break;
        
        case GAP_EVT_ALL_SVC_ADDED:																										//所有的 service 都添加完毕。
        {
            co_printf("All service added\r\n");
            sp_start_adv();
#ifdef USER_MEM_API_ENABLE
            //show_mem_list();
            //show_msg_list();
            //show_ke_malloc();
#endif
        }
        break;

        case GAP_EVT_SLAVE_CONNECT:																										//做为 slave 链接建立。示例：slave[0],connect. link_num:1
        {
						sp_conidx = p_event->param.slave_connect.conidx;
            co_printf("slave[%d],connect. link_num:%d\r\n",p_event->param.slave_connect.conidx,gap_get_connect_num());
						gatt_mtu_exchange_req(p_event->param.slave_connect.conidx);
            gap_conn_param_update(p_event->param.slave_connect.conidx, 6, 6, 0, 500);
        }
        break;

        case GAP_EVT_DISCONNECT:																											//链接断开， 可能是 master 或 slave。示例：Link[0] disconnect,reason:0x13
        {
            co_printf("Link[%d] disconnect,reason:0x%02X\r\n",p_event->param.disconnect.conidx
                      ,p_event->param.disconnect.reason);
            sp_start_adv();
#ifdef USER_MEM_API_ENABLE
            show_mem_list();
            //show_msg_list();
            show_ke_malloc();
#endif
        }
        break;

        case GAP_EVT_LINK_PARAM_REJECT:																								//链接参数更新被拒绝 示例：
            co_printf("Link[%d]param reject,status:0x%02x\r\n"
                      ,p_event->param.link_reject.conidx,p_event->param.link_reject.status);
            break;

        case GAP_EVT_LINK_PARAM_UPDATE:																								//链接参数更新成功。 示例：Link[0]param update,interval:6,latency:0,timeout:500
            co_printf("Link[%d]param update,interval:%d,latency:%d,timeout:%d\r\n",p_event->param.link_update.conidx
                      ,p_event->param.link_update.con_interval,p_event->param.link_update.con_latency,p_event->param.link_update.sup_to);
            break;

        case GAP_EVT_PEER_FEATURE:																										//收到对端的 feature 特性回复 示例：
            co_printf("peer[%d] feats ind\r\n",p_event->param.peer_feature.conidx);
            show_reg((uint8_t *)&(p_event->param.peer_feature.features),8,1);
            break;

        case GAP_EVT_MTU:
            co_printf("mtu update,conidx=%d,mtu=%d\r\n"
                      ,p_event->param.mtu.conidx,p_event->param.mtu.value);
            break;
        
        case GAP_EVT_LINK_RSSI:
            co_printf("link rssi %d\r\n",p_event->param.link_rssi);
            break;
                
        case GAP_SEC_EVT_SLAVE_ENCRYPT:
            co_printf("slave[%d]_encrypted\r\n",p_event->param.slave_encrypt_conidx);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      sp_start_adv
 *					广播函数
 * @brief   Set advertising data & scan response & advertising parameters and start advertising
 *					
 * @param   None. 
 *       
 *
 * @return  None.
 */
static void sp_start_adv(void)
{
    // Set advertising parameters
    gap_adv_param_t adv_param;
    adv_param.adv_mode = GAP_ADV_MODE_UNDIRECT;
    adv_param.adv_addr_type = GAP_ADDR_TYPE_PUBLIC;
    adv_param.adv_chnl_map = GAP_ADV_CHAN_ALL;
    adv_param.adv_filt_policy = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    adv_param.adv_intv_min = 300;
    adv_param.adv_intv_max = 300;
        
    gap_set_advertising_param(&adv_param);
    
    // Set advertising data & scan response data
	gap_set_advertising_data(adv_data, sizeof(adv_data));
	gap_set_advertising_rsp_data(scan_rsp_data, sizeof(scan_rsp_data));
    // Start advertising
	co_printf("Start advertising...\r\n");
	gap_start_advertising(0);															//开始 BLE 广播。需要在设置完广播参数后调用。广播时间到自动停止时会产生 GAP_EVT_ADV_END 事件
}



#ifdef NO_JLINK
/*********************************************************************
 * @fn      timer_refresh_fun
 *
 * @brief   timer_refresh callback function
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void timer_refresh_fun(void *arg)
{
	int32_t temperature, humidity;
	uint8_t LCD_ShowStringBuff[30];
	float capb18_temp =0,capb_airpress=0;
	int8_t ret=0;

	switch (App_Mode)
	{
        case PICTURE_UPDATE:
            break;

        case SENSOR_DATA:
            //SHT30数据读取，并在lcd上显示
            ret = sht3x_measure_blocking_read(&temperature, &humidity);//Read temperature   humidity
            if (ret == STATUS_OK)
            {
                co_printf("temperature = %d,humidity = %d\r\n",temperature,humidity);
                sprintf((char *)LCD_ShowStringBuff,"SHT30_T= %0.1f     ",temperature/1000.0);
                LCD_ShowString(20,140,LCD_ShowStringBuff,BLACK);
                sprintf((char *)LCD_ShowStringBuff,"SHT30_H= %0.1f%%   ",humidity/1000.0);
                LCD_ShowString(20,160,LCD_ShowStringBuff,BLACK);

            }
            else
            {
                co_printf("SHT30 error reading measurement\n");
                
                sprintf((char *)LCD_ShowStringBuff,"SHT30_T= error    ");
                LCD_ShowString(20,140,LCD_ShowStringBuff,BLACK);
                sprintf((char *)LCD_ShowStringBuff,"SHT30_H= error    ");
                LCD_ShowString(20,160,LCD_ShowStringBuff,BLACK);
            }
            //CAPB18数据读取，并在lcd上显示
            ret = CAPB18_data_get(&capb18_temp,&capb_airpress);
            if(ret == true)
            {
                sprintf((char*)LCD_ShowStringBuff,"CAPB18_PRS= %7.5f  ",capb_airpress);
                co_printf("%s\r\n",LCD_ShowStringBuff);
                LCD_ShowString(20,180,LCD_ShowStringBuff,BLACK);

                sprintf((char*)LCD_ShowStringBuff,"CAPB18_TMP= %7.5f  ",capb18_temp);
                co_printf("%s\r\n",LCD_ShowStringBuff);
                LCD_ShowString(20,200,LCD_ShowStringBuff,BLACK);
            }
            else
            {
                co_printf("CAPB18 error reading measurement\n");
                sprintf((char*)LCD_ShowStringBuff,"CAPB18_PRS= error         ");
                co_printf("%s\r\n",LCD_ShowStringBuff);
                LCD_ShowString(20,180,LCD_ShowStringBuff,BLACK);

                sprintf((char*)LCD_ShowStringBuff,"CAPB18_TMP= error         ");
                co_printf("%s\r\n",LCD_ShowStringBuff);
                LCD_ShowString(20,200,LCD_ShowStringBuff,BLACK);
            }
                
            //g-sensor读取，并在lcd上显示
            co_printf("=skip count=%d\r\n",get_skip_num());//获取跳动次数
            sprintf((char*)LCD_ShowStringBuff,"*skip count* = %d",get_skip_num());
            co_printf("%s\r\n",LCD_ShowStringBuff);
            LCD_ShowString(20,120,LCD_ShowStringBuff,BLACK);

            break;
            
        case SPEAKER_FROM_FLASH://播放flash中的音频demo
            sprintf((char*)LCD_ShowStringBuff,"Press the key K2 to start the audio");
            LCD_ShowString(20,100,LCD_ShowStringBuff,BLACK);
            if(!Flash_data_state)
            {
                //如果flash中没有有效的音频数据，则提示如下
                sprintf((char*)LCD_ShowStringBuff,"There is no audio data in flash!");
                LCD_ShowString(20,140,LCD_ShowStringBuff,BLACK);
            }
            else
            {
                LCD_Fill(0,140,240,170,BACK_COLOR);//
            }
            break;
            
        default:
            break;
		
	}
}
#endif


/*********************************************************************
 * @fn      timer_count_fun
 *
 * @brief   计数用定时处理函数.
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
uint16_t timer_count_CNT = 0;
void timer_count_fun(void *arg)
{
	timer_count_CNT++;
	if(timer_count_CNT % 10 == 0)
	{
		if(gpio_porta_read() & GPIO_PIN_1)
		{
			gpio_porta_write(gpio_porta_read() & ~(1<<1));
		}
		else
		{
			gpio_porta_write(gpio_porta_read() | GPIO_PIN_1);
		}
	}
		
}



/*********************************************************************
 * @fn      simple_peripheral_init
 *
 * @brief   Initialize simple peripheral profile, BLE related parameters.
 *					
 * @param   None. 
 *       
 *
 * @return  None.
 */

void simple_peripheral_init(void)
{
    // set local device name
	uint8_t local_name[] = "Simple Peripheral";
	gap_set_dev_name(local_name, sizeof(local_name));

	// Initialize security related settings.
	gap_security_param_t param =
	{
	    .mitm = false,
	    .ble_secure_conn = false,
	    .io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
	    .pair_init_mode = GAP_PAIRING_MODE_WAIT_FOR_REQ,
	    .bond_auth = true,
	    .password = 0,
	};

	gap_security_param_init(&param);																	//初始化安全绑定操作时的参数。

	gap_set_cb_func(app_gap_evt_cb);																	//注册 GAP 事件在应用层的回调函数

	gap_bond_manager_init(BLE_BONDING_INFO_SAVE_ADDR, BLE_REMOTE_SERVICE_SAVE_ADDR, 8, true);	//初始化绑定管理功能。绑定管理功能启用后，会在链接建立回调事件之前进行绑定地址检查。该函数不能在user_entry_before_ble_init() 入口函数调用。
	gap_bond_manager_delete_all();																														//删除所有存储在 flash 内的绑定设备的秘钥和设备服务信息。

	mac_addr_t addr;
	gap_address_get(&addr);																						//获取 ble 设备的 local mac 地址。
	co_printf("Local BDADDR: 0x%2X%2X%2X%2X%2X%2X\r\n", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);

	// Adding services to database
  sp_gatt_add_service();																						//添加GATT service到ATT的数据库里面。 创建ble_simple profile
	speaker_gatt_add_service();				    //创建Speaker profile，
    
	//按键初始化 PD6 PC5
	pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_6), true);
	pmu_set_pin_pull(GPIO_PORT_C, (1<<GPIO_BIT_5), true);
	pmu_port_wakeup_func_set(GPIO_PD6|GPIO_PC5);
	button_init(GPIO_PD6|GPIO_PC5);

	//demo_LCD_APP();							            //显示屏
	//demo_CAPB18_APP();						            //气压计
	//demo_SHT3x_APP();						            //温湿度
	//gyro_dev_init();						            //加速度传感器
	
	//OS Timer
//	os_timer_init(&timer_refresh,timer_refresh_fun,NULL);//创建一个周期性1s定时的系统定时器
//	os_timer_start(&timer_refresh,1000,1);
	os_timer_init(&timer_count,timer_count_fun,NULL);//创建一个周期性100ms定时的系统定时器
	os_timer_start(&timer_count,100,1);
}



