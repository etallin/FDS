/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "fds.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_dfu.h"
#include "MSB8dataencode.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                32                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)              /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)              /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                16384                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                16384                                         /**< UART RX buffer size. */
#define TX_POWER_LEVEL                  (0)    

#define WP1            11   //注意引脚的原始定义
#define WP2            12   //
#define WP3            17   //
#define WP4            18   //
#define SCL            9    //需要将NFC引脚改为gpio引脚
#define SDA            10   //需要将NFC引脚改为gpio引脚

#define TWI_INSTANCE_ID0     0
//#define TWI_INSTANCE_ID1     1
static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi0 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID0);
//static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID1);


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
#define SAMPLES_IN_BUFFER 120
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

//函数声明与全局变量,注意全局变量的值改变
void stop_saadc(void);
void start_saadc(void);
void saadc_sampling_event_enable(void);
void indicaion0_data_send(void);
void indicaion5_data_send(void);
void write_to_flash(void);
bool saadc_enabled = 0;
bool opswitch = 0;
bool channel0_enabled = 1,channel1_enabled = 1,channel2_enabled = 1,channel3_enabled = 1;
int  frequency_sel = 1;
bool writeflash_en = 0;


/** @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
				
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
             NRF_LOG_INFO("Error event: NACK received after sending the address.");
            break;
        
        case NRF_DRV_TWI_EVT_DATA_NACK:
             NRF_LOG_INFO("Error event: NACK received after sending a data byte.");
             break;

        default:
            break;
    }
}
/**@brief TWI initialization.
 */
void twi0_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi0_ADA5251_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    err_code = nrf_drv_twi_init(&m_twi0, &twi0_ADA5251_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi0);
}
/**void twi1_init (void)
//{
//    ret_code_t err_code;

//    const nrf_drv_twi_config_t twi1_ISL9810_config = {
//       .scl                = ARDUINO_SCL_PIN,
//       .sda                = ARDUINO_SDA_PIN,//该引脚
//       .frequency          = NRF_DRV_TWI_FREQ_100K,
//       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//       .clear_bus_init     = false
//    };
//    err_code = nrf_drv_twi_init(&m_twi1, &twi1_ISL9810_config, twi_handler, NULL);
//    APP_ERROR_CHECK(err_code);
//    nrf_drv_twi_enable(&m_twi1);
//}
*/

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)//定时器初始化
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)//GAP初始化
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)//队列写错误操作
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)//连接参数事件
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)//连接参数错误
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)//连接参数初始化
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)//sleep mode
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)//广播事件
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)//蓝牙处理事件
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)//协议栈初始化
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)//GATT事件处理函数
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)//GATT初始化
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */

void bsp_event_handler(bsp_event_t event)//板级处理事件
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
		    case BSP_EVENT_KEY_3:
           if(opswitch)
					 {
							indicaion0_data_send();
						  nrf_gpio_pin_toggle(LED_4);
					 }
					 if(!opswitch && saadc_enabled )
					 {
						 stop_saadc();
						 start_saadc();
						 indicaion5_data_send();
					 }
						break;							

        default:
            break;
    }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) &&
                         (err_code != NRF_ERROR_NOT_FOUND) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
    int8_t        tx_power_level = TX_POWER_LEVEL;//设置发射功率
    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.p_tx_power_level   = &tx_power_level;
	
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every ms/us */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, frequency_sel);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
	  saadc_enabled = 1;
	  NRF_LOG_INFO("SAADC HAL simple example started.");
}

//add fds
#define CONFIG_FILE     (0xF010)
#define CONFIG_REC_KEY  (0x7010)
//static uint8_t data_array1[SAMPLES_IN_BUFFER*2]={0};//fds使用
static uint16_t data_array2[SAMPLES_IN_BUFFER]={0};//调平用的
static uint8_t data_array3[240]={0};//fds使用

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;
/**@brief Function for starting advertising.
 */
static void fds_evt_handler(fds_evt_t const * p_evt)//fds处理事件回调
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
								NRF_LOG_INFO("WRITE RECORD:");
								NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x\n", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
								NRF_LOG_INFO("DELETE RECORD:");
								NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x\n", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}

void record_delete_next(void)
{
    ret_code_t rc;
		do
		{
    fds_find_token_t  tok   = {0};
    fds_record_desc_t desc  = {0};
    rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
    if (rc == FDS_SUCCESS)
    {
        ret_code_t rc = fds_record_delete(&desc);
        if (rc != FDS_SUCCESS)
        {
					NRF_LOG_INFO("DELETE SUCCESS");
        }
    }
    else
    {
        //No records left to delete.
        NRF_LOG_INFO("NO recodes left to be deleted");
			break;
		} 
	}while(rc == FDS_SUCCESS);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{    
	  int product;
		uint16_t val;
	  uint8_t value;
	  ret_code_t err_code;
	  product = channel3_enabled*8 + channel2_enabled*4 + channel1_enabled*2 + channel0_enabled;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {

        static uint8_t data_array[SAMPLES_IN_BUFFER*2];
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        uint8_t i;
        printf("\r\nADC event number:%d \r\n", (int)m_adc_evt_counter);
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
               data_array2[i]=p_event->data.done.p_buffer[i];
					     val = p_event->data.done.p_buffer[i];				
						   printf("%d\r\n",val);					
               //8MSB data				
					     value=val>>4;//right shift	4 bit			
					     value |= 1;//XXXX_XXX1,1 means MSB
							 value &= 247;//XXXX_0XX1,0 means saadc data
/**
					     switch(i|252)
							 {
								 case 252://channel0 8MSB data:XXXX_0001
								 value&=249;
								 break;
								 case 253://channel1 8MSB data:XXXX_0011
								 value&=251;
								 value|=2;
								 break;
								 case 254://channel2 8MSB data:XXXX_0101
								 value|=4;
								 value&=253;
								 break;
								 case 255://channel3 8MSB data:XXXX_0111
								 value|=6;
                 break;								 
							 }
*/
							 value = dataencode(i, product, value);
					     data_array[(i*2)]=value;
							 value=0;
							 
							 //8LSB data,XXXX_XXX0
							 value=val;
					     value&=254;
					     data_array[i*2+1]=value;
               value=0;
				}
        uint16_t length = (uint16_t)(i*2);
				memcpy(data_array3,data_array,length);
				if(!writeflash_en)
				{err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);}
				else
				{write_to_flash();}	
				if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_NOT_FOUND) )
				{
					APP_ERROR_CHECK(err_code);
				}
        m_adc_evt_counter++;
    }
}

/* A record containing dummy configuration data.包含仿真配置数据的记录 
//const定义只读变量，只能在初始化中进行赋值，在后面不能改变
*/
static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,//文件ID
    .key               = CONFIG_REC_KEY,//记录钥匙
    .data.p_data       = &data_array3,//仿真数据配置
    /* The length of a record is always expressed in 4-byte units (words). 数据长度一般超过4个字节*/
    .data.length_words = (sizeof(data_array3)+3) / sizeof(uint32_t),//+3
};

/**static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,//文件ID
    .key               = CONFIG_REC_KEY,//记录钥匙
    .data.p_data       = &data_array3,//仿真数据配置
    //The length of a record is always expressed in 4-byte units (words). 数据长度一般超过4个字节
    .data.length_words = 240,//(sizeof(data_array3) + 3) / sizeof(uint32_t),//240
};
*/

void saadc_init(void)
{
    ret_code_t err_code;
		if(channel0_enabled == 1)								
		{
			nrf_saadc_channel_config_t channel_0_config =
			NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
			err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
			APP_ERROR_CHECK(err_code);			
		}

		if(channel1_enabled == 1)								
		{
			nrf_saadc_channel_config_t channel_1_config =
			NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
			err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
			APP_ERROR_CHECK(err_code);		
		}
		
		if(channel2_enabled == 1)								
		{
			nrf_saadc_channel_config_t channel_2_config =
			NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
			err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
			APP_ERROR_CHECK(err_code);		
		}
		
		if(channel3_enabled == 1)
		{
	  nrf_saadc_channel_config_t channel_3_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
	  err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);
		}
		
		err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
		
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void stop_saadc(void)
{
	nrf_drv_timer_disable(&m_timer);
	nrf_drv_timer_uninit(&m_timer);
	nrf_drv_ppi_channel_disable(m_ppi_channel);
	nrf_drv_ppi_uninit();
	nrf_drv_saadc_abort();
	nrf_drv_saadc_uninit();
	while(nrf_drv_saadc_is_busy());
}

void start_saadc(void)
{
	saadc_sampling_event_init();
	saadc_init();
	saadc_sampling_event_enable();
	m_adc_evt_counter= 0;
}

void indicaion0_data_send(void)
{	
	uint32_t err_code;
	//send specifical string to master,indidcates photoelectric calibration over
	uint16_t length;
	length = 1;
	uint8_t indication0_data[1];
	indication0_data[0] = 0;
	err_code = ble_nus_data_send(&m_nus, indication0_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);
}
void indicaion1_data_send(void)
{
	uint32_t err_code;
	//send specifical string to master,indidcates channel0 can't be balanced
	uint16_t length;
	length = 1;
	uint8_t indication1_data[1];
	indication1_data[0] = 25;
	err_code = ble_nus_data_send(&m_nus, indication1_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);	
}
void indicaion2_data_send(void)
{
	uint32_t err_code;
	//send specifical string to master,indidcates channel1 can't be balanced
	uint16_t length;
	length = 1;
	uint8_t indication2_data[1];
	indication2_data[0] = 41;
	err_code = ble_nus_data_send(&m_nus, indication2_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);	
}
void indicaion3_data_send(void)
{
	uint32_t err_code;
	//send specifical string to master,indidcates channel2 can't be balanced
	uint16_t length;
	length = 1;
	uint8_t indication3_data[1];
	indication3_data[0] = 57;
	err_code = ble_nus_data_send(&m_nus, indication3_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);	
}
void indicaion4_data_send(void)
{
	uint32_t err_code;
	//send specifical string to master,indidcates channel3 can't be balanced
	uint16_t length;
	length = 1;
	uint8_t indication4_data[1];
	indication4_data[0] = 73;
	err_code = ble_nus_data_send(&m_nus, indication4_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);	
}
void indicaion5_data_send(void)
{
	uint32_t err_code;
	//send specifical string to master,indicates restart saadc
	uint16_t length;
	length = 1;
	uint8_t indication5_data[1];
	indication5_data[0] = 89;
	err_code = ble_nus_data_send(&m_nus, indication5_data, &length, m_conn_handle);	
	APP_ERROR_CHECK(err_code);	
}

/**channel leveling
*/
void channel_leveling(void)
{
	bool channe0_enabled_flag = channel0_enabled;
	bool channe1_enabled_flag = channel1_enabled;
	bool channe2_enabled_flag = channel2_enabled;
	bool channe3_enabled_flag = channel3_enabled;	
	ret_code_t err_code;					
	nrf_gpio_cfg_output(WP1);
	nrf_gpio_cfg_output(WP2);
	nrf_gpio_cfg_output(WP3);
	nrf_gpio_cfg_output(WP4);		
	
	//WP1
	while(channe0_enabled_flag)
	{
		nrf_gpio_pin_clear(WP1);
		nrf_gpio_pin_set(WP2);
		nrf_gpio_pin_set(WP3);
		nrf_gpio_pin_set(WP4);
		stop_saadc();
		channel1_enabled = 0; channel2_enabled = 0; channel3_enabled = 0;
		start_saadc();
		int aver,i,a1,a2,a3;
		nrf_drv_saadc_evt_t const * p_event2;
		//two sides of digital potentiometer
		uint8_t salve1_addr = 0x2C;//device1, A1 = 0 A0 = 0;
		uint8_t reg[2];
		reg[1]=0x01;//INSTRUCTIONS BYTE(RDAC1)
		reg[0]=0x00;//data
		err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0x01;
		reg[0]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if((a1>1.65 & a2>1.65)||(a1<1.65 & a2<1.65))
		{
			//send specifical string to master
      indicaion1_data_send();		
			NRF_LOG_INFO("brifge num.1 can not be balanced");	
			break;
		}
		else
		{
			NRF_LOG_INFO("brifge num.1 can be balanced");							
			int sumd=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				reg[0]=0x01;
				sumd=a3>1.65?(sumd-2^(7-j)):(sumd+2^(7-j));
				reg[1]=sumd;
				err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
       }						
		//粗调结束
						
		reg[0]=0x03;//INSTRUCTIONS BYTE(RDAC3)					
		reg[1]=0x00;
		err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if(a1>1.65 & a2>1.65)
		{
		  reg[1]=a2>a1?0x00:0xFF;
			err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else if(a1<1.65 & a2<1.65)
		{
		  reg[1]=a2>a1?0xFF:0x00;
			err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else
		{
			int sumu=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				sumu=a3>1.65?(sumu-2^(7-j)):(sumu+2^(7-j));
				reg[1]=sumu;
				err_code = nrf_drv_twi_tx(&m_twi0, salve1_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
			 }	
			}
			break;
		}
	}//第一路调平结束
	NRF_LOG_INFO("brifge num.1 balanced");
	nrf_gpio_pin_set(WP1);						 

	//WP2
	while(channe1_enabled_flag)
	{
		nrf_gpio_pin_clear(WP2);
		stop_saadc();
		channel0_enabled = 0; channel2_enabled = 0; channel3_enabled = 0;
		start_saadc();
		int aver,i,a1,a2,a3;
		nrf_drv_saadc_evt_t const * p_event2;
		//two sides of digital potentiometer
		uint8_t salve2_addr = 0x2D;//device1, A1 = 0 A0 = 1;
		uint8_t reg[2];
		reg[1]=0x01;//INSTRUCTIONS BYTE(RDAC1)
		reg[0]=0x00;//data
		err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0x01;
		reg[0]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if((a1>1.65 & a2>1.65)||(a1<1.65 & a2<1.65))
		{
			indicaion2_data_send();
			NRF_LOG_INFO("brifge num.2 can not be balanced");	
			break;
		}
		else
		{
			NRF_LOG_INFO("brifge num.2 can be balanced");							
			int sumd=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				reg[0]=0x01;
				sumd=a3>1.65?(sumd-2^(7-j)):(sumd+2^(7-j));
				reg[1]=sumd;
				err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
       }						
		//粗调结束
						
		reg[0]=0x03;//INSTRUCTIONS BYTE(RDAC3)					
		reg[1]=0x00;
		err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if(a1>1.65 & a2>1.65)
		{
		  reg[1]=a2>a1?0x00:0xFF;
			err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else if(a1<1.65 & a2<1.65)
		{
		  reg[1]=a2>a1?0xFF:0x00;
			err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else
		{
			int sumu=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				sumu=a3>1.65?(sumu-2^(7-j)):(sumu+2^(7-j));
				reg[1]=sumu;
				err_code = nrf_drv_twi_tx(&m_twi0, salve2_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
			 }	
			}
			break;
		}
	}//第二路调平结束
	NRF_LOG_INFO("brifge num.2 balanced");
	nrf_gpio_pin_set(WP2);
	
	//WP3
	while(channe2_enabled_flag)
	{
		nrf_gpio_pin_clear(WP3);
		stop_saadc();
		channel0_enabled = 0; channel1_enabled = 0; channel3_enabled = 0;
		start_saadc();
		int aver,i,a1,a2,a3;
		nrf_drv_saadc_evt_t const * p_event2;
		//two sides of digital potentiometer
		uint8_t salve3_addr = 0x2E;//device1, A1 = 1 A0 = 0;
		uint8_t reg[2];
		reg[1]=0x01;//INSTRUCTIONS BYTE(RDAC1)
		reg[0]=0x00;//data
		err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0x01;
		reg[0]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if((a1>1.65 & a2>1.65)||(a1<1.65 & a2<1.65))
		{
			indicaion3_data_send();
			NRF_LOG_INFO("brifge num.3 can not be balanced");	
			break;
		}
		else
		{
			NRF_LOG_INFO("brifge num.3 can be balanced");							
			int sumd=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				reg[0]=0x01;
				sumd=a3>1.65?(sumd-2^(7-j)):(sumd+2^(7-j));
				reg[1]=sumd;
				err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
       }						
		//粗调结束
						
		reg[0]=0x03;//INSTRUCTIONS BYTE(RDAC3)					
		reg[1]=0x00;
		err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if(a1>1.65 & a2>1.65)
		{
		  reg[1]=a2>a1?0x00:0xFF;
			err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else if(a1<1.65 & a2<1.65)
		{
		  reg[1]=a2>a1?0xFF:0x00;
			err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else
		{
			int sumu=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				sumu=a3>1.65?(sumu-2^(7-j)):(sumu+2^(7-j));
				reg[1]=sumu;
				err_code = nrf_drv_twi_tx(&m_twi0, salve3_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
			 }	
			}
			break;
		}
	}//第三路调平结束
	NRF_LOG_INFO("brifge num.3 balanced");
	nrf_gpio_pin_set(WP3);
	
	//WP4
	while(channe3_enabled_flag)
	{
		nrf_gpio_pin_clear(WP3);
		stop_saadc();
		channel0_enabled = 0; channel1_enabled = 0; channel2_enabled = 0;
		start_saadc();
		int aver,i,a1,a2,a3;
		nrf_drv_saadc_evt_t const * p_event2;
		//two sides of digital potentiometer
		uint8_t salve4_addr = 0x2F;//device1, A1 = 1 A0 = 1;
		uint8_t reg[2];
		reg[1]=0x01;//INSTRUCTIONS BYTE(RDAC1)
		reg[0]=0x00;//data
		err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0x01;
		reg[0]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if((a1>1.65 & a2>1.65)||(a1<1.65 & a2<1.65))
		{
			indicaion4_data_send();
			NRF_LOG_INFO("brifge num.4 can not be balanced");	
			break;
		}
		else
		{
			NRF_LOG_INFO("brifge num.4 can be balanced");							
			int sumd=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				reg[0]=0x01;
				sumd=a3>1.65?(sumd-2^(7-j)):(sumd+2^(7-j));
				reg[1]=sumd;
				err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
       }						
		//粗调结束
						
		reg[0]=0x03;//INSTRUCTIONS BYTE(RDAC3)					
		reg[1]=0x00;
		err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a1=aver*3.6/4096;										
		}
		reg[1]=0xFF;
		err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
		APP_ERROR_CHECK(err_code);
		if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
		{
			int sum=0;
			for(i=0;i<120;i++)
			{
				sum=sum+data_array2[i];
			}
			aver=sum/120;
			a2=aver*3.6/4096;										
		}
		if(a1>1.65 & a2>1.65)
		{
		  reg[1]=a2>a1?0x00:0xFF;
			err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else if(a1<1.65 & a2<1.65)
		{
		  reg[1]=a2>a1?0xFF:0x00;
			err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
			APP_ERROR_CHECK(err_code);
			break;
			//结束
		}
		else
		{
			int sumu=a2>a1?256:0;
			a3=a2>a1?a2:a1;						
			for(int j=0;j<8;j++)
			{							
				sumu=a3>1.65?(sumu-2^(7-j)):(sumu+2^(7-j));
				reg[1]=sumu;
				err_code = nrf_drv_twi_tx(&m_twi0, salve4_addr, reg, sizeof(reg), false);
				APP_ERROR_CHECK(err_code);
				if(m_xfer_done == true && p_event2->type == NRF_DRV_SAADC_EVT_DONE);
				{
					int sum=0;
					for(i=0;i<120;i++)
					{
						sum=sum+data_array2[i];
					}
					aver=sum/120;
					a3=aver*3.6/4096;										
				}					
			 }	
			}
			break;
		}
	}//第四路调平结束
	NRF_LOG_INFO("brifge num.4 balanced");
	nrf_gpio_pin_set(WP4);
	
	stop_saadc();
	channel0_enabled = channe0_enabled_flag;
	channel1_enabled = channe1_enabled_flag;
	channel2_enabled = channe2_enabled_flag;
	channel3_enabled = channe3_enabled_flag;
	start_saadc();	
}



/**write to flash*/
uint32_t *data;
fds_record_desc_t desc;
fds_find_token_t  tok;
void fds_test(void)
{
	ret_code_t rc;
	(void) fds_register(fds_evt_handler);//FDS注册
	rc = fds_init();//fds初始化
	if(rc == !FDS_SUCCESS)
		{NRF_LOG_INFO("fds init failed");}
	else
		APP_ERROR_CHECK(rc);
					
	while (!m_fds_initialized)//等待初始化完成
	{
		NRF_LOG_INFO("fds init failed");
		sd_app_evt_wait();//等待过程中待机
	}
}
void clear_fds(void)
{
	ret_code_t rc;
	fds_stat_t stat = {0};
	rc = fds_stat(&stat);//设置统计数据
	if(rc == FDS_ERR_NOT_INITIALIZED)
	{
		NRF_LOG_INFO("FDS_ERR_NOT_INITIALIZED");
	}
	else if(rc == FDS_ERR_NULL_ARG)
	{
		NRF_LOG_INFO("FDS_ERR_NULL_ARG");		
	}
	else
	APP_ERROR_CHECK(rc);
	
	NRF_LOG_INFO("pages_available:%d",stat.pages_available);
	NRF_LOG_INFO("open_records:%d",stat.open_records);
	NRF_LOG_INFO("valid_records:%d",stat.valid_records);
	NRF_LOG_INFO("dirty_records:%d",stat.dirty_records);
	NRF_LOG_INFO("words reserved:%d",stat.words_reserved);
	NRF_LOG_INFO("words used:%d",stat.words_used);	
	NRF_LOG_INFO("largest contig:%d",stat.largest_contig);

	record_delete_next();//把所有记录清零,fds_record_delete函数并不会释放记录占用的内存,需使用fds_gc进行垃圾回收;
	rc = fds_gc();//内存回收
	if(rc == FDS_SUCCESS)
	{
		NRF_LOG_INFO("GC SUCCESS");
	}
	else
		APP_ERROR_CHECK(rc);
	writeflash_en = 1;
}

void write_to_flash(void)
{
			fds_record_desc_t desc = {0};//用来操作记录的描述符结构清零
//			fds_find_token_t  tok  = {0};//保存秘钥的令牌清零
			ret_code_t rc;
			rc = fds_record_write(&desc, &m_dummy_record);
			if(rc == FDS_ERR_NO_SPACE_IN_FLASH)
			{			
				
				stop_saadc();
				writeflash_en = 0;
				ret_code_t err_code;
				uint16_t length;
				length = 2;
				uint8_t flash_full[2];
				flash_full[0] = 105;
				flash_full[1] = 105;
				err_code = ble_nus_data_send(&m_nus, flash_full, &length, m_conn_handle);
				if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_NOT_FOUND) )
				{
					APP_ERROR_CHECK(err_code);
				}
				NRF_LOG_INFO("NO SPACE IN FLASH, FDS OPERATION IS OVER")			
			}
			if( rc== FDS_ERR_NO_SPACE_IN_QUEUES)
			{
				writeflash_en = 0;
				stop_saadc();	
				NRF_LOG_INFO("NO SPACE IN QUEUES, FDS OPERATION IS OVER");
			}
}

/**read flash*/
void read_from_flash()
{
	ret_code_t rc;
//	fds_record_desc_t desc = {0};//用来操作记录的描述符结构清零
	fds_find_token_t  tok  = {0};//保存秘钥的令牌清零
//	do
//	{
		rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);//对应KEY记录查找数据			
		if (rc == FDS_SUCCESS)//如果查找成功
		{
			/* A config file is in flash. Let's update it. */
			fds_flash_record_t config = {0};//把配置清零

			/* Open the record and read its contents. */
			rc = fds_record_open(&desc, &config);//打开记录读取数据
			APP_ERROR_CHECK(rc);
								
			NRF_LOG_INFO("Found Record ID = %d",desc.record_id);
			NRF_LOG_INFO("Data = ");
			data = (uint32_t *)config.p_data;
//			static uint8_t data_array_send[4];
			for (uint16_t i=0; i<config.p_header->length_words; i++)//i<config.p_header->length_words
			{
				NRF_LOG_INFO("0x%8x",data[i]);//打印输出数据
//				data_array_send[0] = data[i];
//				data_array_send[1] = data[i]>>8;
//				data_array_send[2] = data[i]>>16;
//				data_array_send[3] = data[i]>>24;
//				uint16_t length1= 4;
//				ret_code_t err_code;
//				err_code = ble_nus_data_send(&m_nus, data_array_send, &length1, m_conn_handle);
//				if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_NOT_FOUND) )
//				{
//					APP_ERROR_CHECK(err_code);
//				}		
			}
			NRF_LOG_INFO("\r\n");					
		}
//		if(rc == FDS_ERR_NULL_ARG)
//		{
//			NRF_LOG_INFO("desc or token is empty");
//		}
//		if(rc == FDS_ERR_NOT_FOUND)
//		{
//			NRF_LOG_INFO("can't find record ID");
//		}
		else
		{
			NRF_LOG_INFO("can't find record or no more record to be read");
			
			//send specifical string to master,indidcates flash read end
//			ret_code_t err_code;
//			uint16_t length;
//			length = 2;
//			uint8_t flash_read_end[2];
//			flash_read_end[0] = 121;
//			flash_read_end[1] = 121;
//			err_code = ble_nus_data_send(&m_nus, flash_read_end, &length, m_conn_handle);	
//			if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_NOT_FOUND) )
//			{
//				APP_ERROR_CHECK(err_code);
//			}	
//			break;
		}
//	}while(rc == FDS_SUCCESS);
	/* Close the record when done reading. */
	rc = fds_record_close(&desc);//关闭记录
	if(rc == NRF_SUCCESS)
	{NRF_LOG_INFO("flash close success");}
	else
	{NRF_LOG_INFO("flash close fail");}
	writeflash_en = 0;
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)//串口中断操作
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
			    //光电开关校准开始,此时并不采集数据
    			if(p_evt->params.rx_data.p_data[0]==11)  
					{
						opswitch = 1;					
					}			    
			    //校准完成
					if(p_evt->params.rx_data.p_data[0]==12)  
					{
						opswitch = 0;
					}
			    //start saadc
			    if(p_evt->params.rx_data.p_data[0]==13)  
					{
            start_saadc();
					}
					//stop saadc
					if(p_evt->params.rx_data.p_data[0]==14)  
					{
						stop_saadc();
					}
					
					//set channel & frequency
					if(p_evt->params.rx_data.p_data[0]==15)  
					{					
							for(int i=1;i<6;i++)
							{
								switch(i)
								{
										case 1:
											if(p_evt->params.rx_data.p_data[1]==49)
												channel0_enabled = 1;
											else
												channel0_enabled = 0;
											break;
										case 2:
											if(p_evt->params.rx_data.p_data[2]==49)
												channel1_enabled = 1;	
											else
												channel1_enabled = 0;	
											break;
										case 3:
											if(p_evt->params.rx_data.p_data[3]==49)
												channel2_enabled = 1;
											else
												channel2_enabled = 0;
											break;
										case 4:
											if(p_evt->params.rx_data.p_data[4]==49)
												channel3_enabled = 1;
											else
												channel3_enabled = 0;
											break;
									  case 5:
											if(p_evt->params.rx_data.p_data[5] == 48)//100HZ,需要使用labview实测一下,60HZ
											{frequency_sel = 40;}
											else if(p_evt->params.rx_data.p_data[5] == 49)//500HZ
											{frequency_sel = 8;}
											else if(p_evt->params.rx_data.p_data[5] == 50)//1000HZ
											{frequency_sel = 4;}
											else if(p_evt->params.rx_data.p_data[5] == 51)//1500HZ
											{frequency_sel = 3;}
											else if(p_evt->params.rx_data.p_data[5] == 52)//2000HZ
											{frequency_sel = 2;}
											else if(p_evt->params.rx_data.p_data[5] == 53)//4000HZ
											{frequency_sel = 1;}
											else
											{frequency_sel = 1;}
											break;
											
										default:
											break;
								}
						  }
			    }
					
					if(p_evt->params.rx_data.p_data[0]==16)  
					{
						channel_leveling();
					}					

					if(p_evt->params.rx_data.p_data[0]==17)  
					{
						NRF_LOG_INFO("FDS OPERATION");
//						clear_fds();
						writeflash_en = 1;
					}
					
					if(p_evt->params.rx_data.p_data[0]==18)  
					{
						read_from_flash();
					}					
		}
}
/**@snippet [Handling the data received over BLE] */
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)//服务初始化
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**connection event length extension*/
void conn_evt_len_ext_set()
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = 1;
	  err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}



/**@brief Application main function.
 */
int main(void)
{
    // Initialize.
		bool erase_bonds;
	  twi0_init();
//    twi1_init(); 
    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
		fds_test();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
	  conn_evt_len_ext_set();
   
	  // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
	  tx_power_set();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

/**
 * @}
 */
