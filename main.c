/*! \file main.c
    \brief 
    
    
*/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_util_platform.h"

// Logger Settings and includes
#define NRF_LOG_MODULE_NAME "App"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// app_timer settings and includes
#include "app_timer.h"

// Scheduler settings and includes
#include "app_scheduler.h"
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof (app_timer_event_t)
#define SCHED_QUEUE_SIZE                10

// Nordic
#include "nrf_drv_twi.h"
#include "app_twi.h"
#include "nrf_drv_pdm.h"
#include "app_button.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "softdevice_handler_appsh.h"
#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "ble_nus.h"

// Application Stuff
#include "hardware.h"
#include "GPIOHandler.h"
#include "BlinkLib.h"

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static nrf_ble_gatt_t m_gatt;

static ble_nus_t m_nus;

// Defines
// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_SYNTH,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define CUSTOM_UUID { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 }

/* BLE Things */
#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Blink"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "TL"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      30                                         /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

								 
typedef enum {
	ALL_DISABLED,
	FX_LIBRARY_MODES,
	SOUND_REACTIVE,
} blinkensteinMode_t;

static volatile blinkensteinMode_t currentMode = ALL_DISABLED;

static void set_mode(blinkensteinMode_t mode)
{
	if (currentMode == mode) return;
	
	switch (mode)
	{
		case ALL_DISABLED:
			break;
		case FX_LIBRARY_MODES:
			break;
		case SOUND_REACTIVE:
			break;
	}
}

/*! \fn static void log_intialize(void)
    \brief Initializes Nordic Logging module.
*/
static void log_initialize(void)
{
	uint32_t err_code;
	
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	
	NRF_LOG_INFO("\r\n\r\n\r\n\r\n\r\n");
	NRF_LOG_INFO("******************\r\n");
	NRF_LOG_INFO("\tBlinkenstein!\r\n");
	NRF_LOG_INFO("******************\r\n");
	NRF_LOG_FLUSH();
}

/*! \fn static void scheduler_initialize(void)
    \brief Initializes Nordic App_Scheduler module.
*/
static void scheduler_initialize(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/*! \fn static void app_timer_initialize(void)
    \brief Initializes Nordic App_Timer module.
*/
static void app_timer_initialize(void)
{
	uint32_t err_code;
	
	err_code = app_timer_init();
	
	APP_ERROR_CHECK(err_code);
}

/*! \fn static void power_manage(void)
    \brief Puts device into sleep state to wait for an event.
*/
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void pdm_event_handler(uint32_t * p_buffer, uint16_t length)
{
	// process
	NRF_LOG_HEXDUMP_INFO(p_buffer, 10);
}

int16_t mic_buffer[2][512];
void mic_initialize(void)
{
	nrf_drv_pdm_config_t pdm_cfg = NRF_DRV_PDM_DEFAULT_CONFIG(MIC_CLK_PIN, MIC_DATA_PIN, mic_buffer[0], mic_buffer[1], 512);
	
	APP_ERROR_CHECK(nrf_drv_pdm_init(&pdm_cfg, pdm_event_handler));
}

#define BUTTON_DEBOUNCE_DELAY 50

APP_TIMER_DEF(buttonTimer);

static void gpiote_scheduler_evt_handler(void * p_event_data, uint16_t event_size)
{
	uint8_t pin = *(uint8_t*)p_event_data;
	
	switch (pin)
	{
		
	}
}

static void gpiote_drv_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	switch (action)
	{
		case NRF_GPIOTE_POLARITY_HITOLO:
			break;
		case NRF_GPIOTE_POLARITY_LOTOHI:
			APP_ERROR_CHECK(app_sched_event_put(&pin, sizeof pin, gpiote_scheduler_evt_handler));
			break;
		case NRF_GPIOTE_POLARITY_TOGGLE:
			break;
	}
}

void gpiote_initialize(void)
{
	if (!nrf_drv_gpiote_is_init())
	{
		APP_ERROR_CHECK(nrf_drv_gpiote_init());
	}
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	in_config.pull = NRF_GPIO_PIN_NOPULL;
	
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ACCEL_INT1_PIN, &in_config, gpiote_drv_event_handler));
	
	nrf_drv_gpiote_in_event_enable(ACCEL_INT1_PIN, true);
}

static volatile int32_t buttonPresses = 0;

static void app_button_interrupt_handler(uint8_t pin, uint8_t action)
{
	switch (action)
	{
		case APP_BUTTON_PUSH:
			if (pin == BUTTON_PIN)
			{
				NRF_LOG_INFO("Button pressed\r\n");
				buttonPresses++;
				app_timer_stop(buttonTimer);
				app_timer_start(buttonTimer, APP_TIMER_TICKS(500), NULL);
			}
			break;
		case APP_BUTTON_RELEASE:
			break;
	}
}


static void buttonTimerTimeoutHandler(void * p_context)
{
	if (app_button_is_pushed(0))
	{
		// hold
		buttonPresses *= (-1);
		app_timer_start(buttonTimer, APP_TIMER_TICKS(500), NULL);
	}
	else
	{
		NRF_LOG_INFO("Button pressed: %i\r\n", buttonPresses);
		switch (buttonPresses)
		{
			case 1:
				// Next Mode
				break;
			case 5:
				// Lights out
			default:
				break;
		}
		buttonPresses = 0;
	}
}

void app_button_initialize(void)
{
	static app_button_cfg_t p_button = { BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, app_button_interrupt_handler };
	
	APP_ERROR_CHECK(app_button_init(&p_button, 1, BUTTON_DEBOUNCE_DELAY));
	APP_ERROR_CHECK(app_button_enable());
	
	APP_ERROR_CHECK(app_timer_create(&buttonTimer, APP_TIMER_MODE_SINGLE_SHOT, buttonTimerTimeoutHandler));
}

void on_adv_err(uint32_t err_code)
{
	NRF_LOG_DEBUG("Adv module error: %x\r\n", err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void advertising_start(void)
{
	APP_ERROR_CHECK(ble_advertising_start(BLE_ADV_MODE_FAST));
}

static void sleep_mode_enter(void)
{
	APP_ERROR_CHECK(sd_power_system_off());
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising\r\n");
            break;

        case BLE_ADV_EVT_IDLE:
			//advertising_start();
			sleep_mode_enter();
            break;

        default:
            break;
    }
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;
			memset(&auth_reply, 0, sizeof(auth_reply));

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;
			
            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL) ||
					(req.request.write.op == BLE_GATTS_OP_WRITE_REQ)) // We added this here. I'm not 100% sure this is where we should be performing this action.
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    //auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
					auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
					auth_reply.params.write.p_data	=	p_ble_evt->evt.gatts_evt.params.write.data;
					auth_reply.params.write.len		=	p_ble_evt->evt.gatts_evt.params.write.len;
					auth_reply.params.write.offset	=	p_ble_evt->evt.gatts_evt.params.write.offset;
					auth_reply.params.write.update	=	1;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
	fs_sys_event_handler(sys_evt);
	
	ble_advertising_on_sys_evt(sys_evt);
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
	nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
	
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
}

static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);
	
	// RAM Start
	uint32_t ram_start = 0;
	err_code = softdevice_app_ram_start_get(&ram_start);
	APP_ERROR_CHECK(err_code);
	
	// BLE Configuration
	ble_cfg_t ble_cfg;
	
	// Max Connections
	memset(&ble_cfg, 0, sizeof ble_cfg);
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
	ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count = 0;
	err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
	// Print MAC
	//ble_gap_addr_t ble_addr;
	//err_code = sd_ble_gap_addr_get(&ble_addr);
	//NRF_LOG_DEBUG("MAC Address: %X:%X:%X:%X:%X:%X\r\n", ble_addr.addr[0], ble_addr.addr[1], ble_addr.addr[2], ble_addr.addr[3], ble_addr.addr[4], ble_addr.addr[5]);
}

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_DEBUG("Connected to previously bonded device\r\n");
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_START:
            break; // PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_DEBUG("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:
        {
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            switch (p_evt->params.conn_sec_failed.error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    // Rebond if one party has lost its keys.
                    err_code = pm_conn_secure(p_evt->conn_handle, true);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    break; // PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

                default:
                    break;
            }
        } break; // PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break; // PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break; // PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break; // PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break; // PM_EVT_PEER_DATA_UPDATE_FAILED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break; // PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break; // PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            //advertising_start();
            break; // PM_EVT_PEERS_DELETE_SUCCEEDED

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break; // PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break; // PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break; // PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        default:
            // No implementation needed.
            break;
    }
}

static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
	
	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
	APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t				err_code;
    ble_advdata_t			advdata;
    ble_adv_modes_config_t	options;
	
	// Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               		=	BLE_ADVDATA_FULL_NAME;
	advdata.short_name_len					=	4;
    advdata.include_appearance  		    =	false;
    advdata.flags                  			=	BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.p_manuf_specific_data			=	NULL;
	advdata.include_ble_device_addr			=	true;
	//advdata.le_role							=	NULL;
	advdata.p_tk_value						=	NULL;
	advdata.p_sec_mgr_oob_flags				=	NULL;
	
	
    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
	
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, on_adv_err);
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_ERROR("BLE Advertising Module initialization failed! err: %x\r\n", err_code);
	}
    APP_ERROR_CHECK(err_code);
}

static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	NRF_LOG_HEXDUMP_INFO(p_data, length);
}

static void services_init(void)
{
	ble_nus_init_t nus_init;
	memset(&nus_init, 0, sizeof nus_init);
	
	nus_init.data_handler = nus_data_handler;
	
	APP_ERROR_CHECK(ble_nus_init(&m_nus, &nus_init));
}

static void conn_params_init(void)
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

static void gatt_init(void)
{
	APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, NULL));
}

void ble_initialize(void)
{
	ble_stack_init();
	peer_manager_init(false);
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	advertising_start();
}

int main(void)
{
	log_initialize();
	scheduler_initialize();
	app_timer_initialize();
	
	//gpiote_initialize();
	app_button_initialize();
	ble_initialize();
	
	//mic_initialize();
	initializeLeds();
	renderLeds();
	
	NRF_LOG_INFO("Start!\r\n");
	for (;;)
	{
		app_sched_execute();
		if (NRF_LOG_PROCESS() == false)
		{
			power_manage();
		}
	}
}
