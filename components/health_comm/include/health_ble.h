#ifndef _HEALTH_BLE_H_
#define _HEALTH_BLE_H_


#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

   

    SPP_IDX_NB,
};




esp_err_t  health_ble_send(uint8_t *pdata,uint16_t plen);



esp_err_t  health_ble_deinit();

esp_err_t  health_ble_init(QueueHandle_t xQueue_Req);

#endif

