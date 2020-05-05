/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
//#include "nvs_flash.h"
#include "esp_bt.h"

#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "driver/timer.h"

#include "health_hw.h"
#include "health_comm.h"
#include "health_comm_protocol.h"


#include "health_ble.h"

#define GATTS_TABLE_TAG  "FeelKit_BLE"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "FeelKit"
#define SPP_SVC_INST_ID             0

/// SPP Service
static const uint16_t spp_service_uuid =  0x0001    ;        // 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE     0x0002   // 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY      0x0003  // 0xABF2
//#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
//#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#define TEST_TIMER_ID           (0)

/*
static const uint8_t spp_adv_data[23] = {
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x0F,0x09,0x45,0x53,0x50,0x5f,0x53,0x50,0x50,0x5f,0x53,0x45,0x52,0x56,0x45,0x52
};
*/
static const uint8_t spp_adv_data[23] =
{
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x08,0x09,0x46,0x65,0x65,0x6C,0x4B,0x69,0x74
};
static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;


static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static hsensor_comm_protocol_t ble_rx_pc;


static esp_ble_adv_params_t spp_adv_params =
{
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};



static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] =
{
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;


///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
//static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
//static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
//static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
//static const uint8_t  spp_status_val[10] = {0x00};
//static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};


///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                       =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid
        }
    },

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write
        }
    },

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]                 =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
            SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val
        }
    },

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify
        }
    },

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
            SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val
        }
    },

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {   {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
            sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc
        }
    },

    //SPP -  command characteristic Declaration
//   [SPP_IDX_SPP_COMMAND_CHAR]            =
//    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
//    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
//    [SPP_IDX_SPP_COMMAND_VAL]                 =
//    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
//    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    // [SPP_IDX_SPP_STATUS_CHAR]            =
    // {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    // CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
//   [SPP_IDX_SPP_STATUS_VAL]                 =
//   {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
//   SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
//    [SPP_IDX_SPP_STATUS_CFG]         =
//    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
//   sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++)
    {
        if(handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
            }
            break;
        default:
            break;
    }
}

static void tmp_send_test_data(void)
{
esp_err_t ret = ESP_ERR_INVALID_STATE;

        uint8_t tmp[20],crc,i;
        uint16_t tval;
        tmp[0] = 0xaa;
        tmp[1] = 0xbb;
        tmp[2] = 16;
        tval = (rand()%7+34)*100;
        tmp[3] = ( uint8_t )(tval>>8);
        tmp[4] = ( uint8_t )(tval);
        tval = (rand()%80+55)*100;
        tmp[5] = ( uint8_t )(tval>>8);
        tmp[6] = ( uint8_t )(tval);
        tval = (rand()%30+5)*100;
        tmp[7] = ( uint8_t )(tval>>8);
        tmp[8] = ( uint8_t )(tval);
        tval = (rand()%64)*100;
        tmp[9] = ( uint8_t )(tval>>8);
        tmp[10] = ( uint8_t )(tval);
        tval = rand()%1000+100;
        tmp[11] = ( uint8_t )(tval>>8);
        tmp[12] = ( uint8_t )(tval);
        tmp[13] = 0;
        tmp[14] = 0;
        tmp[15] = 0;
        tmp[16] = 0;
        tmp[17] = 0;
        tmp[18] = 0;
        crc =0;
        for(i=0; i<20; i++)
        {
            crc ^= tmp[i];
        }
        tmp[19] = crc;
        //  health_ble_send(tmp,20);
        if(!enable_data_ntf)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
        }
        else
        {
            ret = esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],20, tmp, false);
            if(ret != ESP_OK)
                ESP_LOGE(GATTS_TABLE_TAG, "%s send error\n", __func__);
        }

    

}

static void tmp_test_watch(uint8_t *src,uint8_t len)
{
    if((src[0] == 0xaa)&&(src[1] == 0xbb)&&(src[2] == 1)&&(src[3] == 0x58))
    {
        tmp_send_test_data();
    }

}

static void IRAM_ATTR htest_timer_isr(void *para)
{
    int timer_idx = (int) para;
    hdata_type_t evt ;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG1.int_st_timers.val;


    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
    {
        TIMERG1.int_clr_timers.t0 = 1;
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
 tmp_send_test_data();
 
}


static void htest_setup_timer()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = 16;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TIMER_AUTORELOAD_EN;
    timer_init(TIMER_GROUP_1, TEST_TIMER_ID, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, TEST_TIMER_ID, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_1, TEST_TIMER_ID,5 * (TIMER_BASE_CLK / 16)  );
    timer_enable_intr(TIMER_GROUP_1,TEST_TIMER_ID);
    timer_isr_register(TIMER_GROUP_1, TEST_TIMER_ID, htest_timer_isr, (void *) TEST_TIMER_ID, ESP_INTR_FLAG_IRAM, NULL);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

            ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

            ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
            esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            break;
        case ESP_GATTS_WRITE_EVT:
        {
            res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false)
            {
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_DATA_NTF_CFG)
                {
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00))
                    {
                        enable_data_ntf = true;
						timer_start(TIMER_GROUP_1 , TEST_TIMER_ID);
                    }
                    else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00))
                    {
                        enable_data_ntf = false;
						timer_pause(TIMER_GROUP_1 , TEST_TIMER_ID);
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL)
                {
                    for(uint16_t i=0; i<p_data->write.len; i++)
                        hcomm_RecvFrame( &ble_rx_pc,p_data->write.value[i]);
                    tmp_test_watch(p_data->write.value,p_data->write.len);
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
                }
                else
                {
                    //TODO:
                }
            }
            else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL))
            {
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");

            break;
        }
        case ESP_GATTS_MTU_EVT:
            spp_mtu_size = p_data->mtu.mtu;
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            spp_conn_id = p_data->connect.conn_id;
            spp_gatts_if = gatts_if;
            is_connected = true;
            memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            is_connected = false;
            enable_data_ntf = false;
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
            }
            else
            {
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;
        }
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    }
    while (0);
}

esp_err_t  health_ble_send(uint8_t *pdata,uint16_t plen)

{
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    uint16_t slen;
    uint8_t mtu,length = plen,i=0;
    mtu = spp_mtu_size - 3;

    if(!enable_data_ntf)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
    }
    else
    {

        do
        {
            if(length >= mtu)
            {
                slen = mtu;
                length -= mtu;
            }
            else
            {
                slen = length;
                length = 0;
            }
            ret = esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],slen, pdata+ i*mtu, false);
            if(ret != ESP_OK)
                break;
            else
                i++;

            vTaskDelay(20 / portTICK_PERIOD_MS);


        }
        while(length > 0);
    }
    return ret;
}

esp_err_t  health_ble_deinit()
{
    return ESP_OK;
}

static QueueHandle_t ble_cb_queue;

static esp_err_t  health_ble_framecb(uint8_t *dframe)
{
    hdata_type_t creq;

    creq.type = REQ_FROM_BLE;
    creq.pdata = (void*)dframe;

    xQueueSend (ble_cb_queue, (void * )&creq, 0);
    return 0;
}

esp_err_t  health_ble_init(QueueHandle_t xQueue_Req)
{
    esp_err_t ret = ESP_OK;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);
    ble_rx_pc.frame_cb = health_ble_framecb;
    ble_cb_queue = xQueue_Req;
    hcomm_RecvInit(&ble_rx_pc);
htest_setup_timer();

    return ret;
}
