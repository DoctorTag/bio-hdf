/*
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "esp_system.h"
#include "health_board.h"

#include "health_hw.h"
#include "health_comm_protocol.h"

#include "health_comm.h"

#include "local_uart.h"
#include "health_tcp.h"
#include "health_ble.h"

#ifndef __FILENAME__
#define __FILENAME__ __FILE__
#endif



#define HEALTH_COMM_MEM_CHECK(TAG, a, action) if (!(a)) {                                       \
        ESP_LOGE(TAG,"%s:%d (%s): %s", __FILENAME__, __LINE__, __FUNCTION__, "Memory exhausted");       \
        action;                                                                   \
        }
#define HEALTH_COMM_ERROR(TAG, str) ESP_LOGE(TAG, "%s:%d (%s): %s", __FILENAME__, __LINE__, __FUNCTION__, str)

#define HEALTH_COMM_CHECK_NULL(a, format, b, ...) \
    if ((a) == 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }


#define mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS);
#define mutex_unlock(x)     xSemaphoreGive(x)
#define mutex_create()      xSemaphoreCreateMutex()
#define mutex_destroy(x)    vSemaphoreDelete(x)



static const char *TAG = "FeelKit_Comm";

struct hsensor_comm
{
    esp_err_t (*hsensor_comm_initialize)(QueueHandle_t xQueue_Req);
    esp_err_t (*hsensor_comm_deinitialize)(void);
    esp_err_t (*hsensor_comm_send)(uint8_t *pdata,uint16_t plen);
//   esp_err_t (*hsensor_comm_receive)(hsensor_comm_mode_t mode, hsensor_comm_codec_i2s_iface_t *iface);
//   esp_err_t (*hsensor_comm_ctrl)(hsensor_comm_mode_t mode,int volume);
    xSemaphoreHandle hsensor_comm_lock;
    void *handle;
};

static struct hsensor_comm hsensor_comm_default[] =
{

    {
        .hsensor_comm_initialize = health_tcp_init,
        .hsensor_comm_deinitialize = health_tcp_deinit,
        // .hsensor_comm_ctrl = es8374_ctrl_state,
        .hsensor_comm_send = health_tcp_send,
    }

    ,
    {
        .hsensor_comm_initialize = health_ble_init,
        .hsensor_comm_deinitialize = health_ble_deinit,
        // .hsensor_comm_ctrl = zl38063_ctrl_state,
        .hsensor_comm_send = health_ble_send,
    }
#ifdef CONFIG_HLOCAL_UART
    ,
    {
        .hsensor_comm_initialize = local_uart_init,
        .hsensor_comm_deinitialize =  local_uart_deinit,
//        .hsensor_comm_ctrl = es8388_ctrl_state,
        .hsensor_comm_send = local_uart_send,
    }
#endif
};

hsensor_comm_handle_t hsensor_comm_init(QueueHandle_t xQueue, int index)
{

    if (NULL != hsensor_comm_default[index].handle)
    {
        ESP_LOGW(TAG,"The hal has been already initialized!");
        return hsensor_comm_default[index].handle;
    }
    hsensor_comm_handle_t hsensor_comm = (hsensor_comm_handle_t) calloc(1, sizeof(struct hsensor_comm));
    HEALTH_COMM_MEM_CHECK(TAG, hsensor_comm, return NULL);
    memcpy(hsensor_comm, &hsensor_comm_default[index], sizeof(struct hsensor_comm));
    hsensor_comm->hsensor_comm_lock = mutex_create();

    HEALTH_COMM_MEM_CHECK(TAG, hsensor_comm->hsensor_comm_lock,
    {
        free(hsensor_comm);
        return NULL;
    });
    mutex_lock(hsensor_comm->hsensor_comm_lock);
    hsensor_comm->hsensor_comm_initialize(xQueue);
    hsensor_comm->handle = hsensor_comm;
    hsensor_comm_default[index].handle = hsensor_comm;
    mutex_unlock(hsensor_comm->hsensor_comm_lock);

    ESP_LOGI(TAG,"The hal has been initialized!");
    return hsensor_comm;
}

esp_err_t hsensor_comm_deinit(hsensor_comm_handle_t hsensor_comm, int index)
{
    esp_err_t ret;
    HEALTH_COMM_CHECK_NULL(hsensor_comm, "hsensor_comm handle is null", -1);
    mutex_destroy(hsensor_comm->hsensor_comm_lock);
    ret = hsensor_comm->hsensor_comm_deinitialize();
    hsensor_comm->hsensor_comm_lock = NULL;
    hsensor_comm->handle = NULL;
    hsensor_comm_default[index].handle = NULL;
    free(hsensor_comm);
    hsensor_comm = NULL;
    return ret;
}

/*
esp_err_t hsensor_comm_ctrl(hsensor_comm_handle_t hsensor_comm, audio_hal_codec_mode_t mode, audio_hal_ctrl_t audio_hal_state)
{
    esp_err_t ret;
    AUDIO_HAL_CHECK_NULL(hsensor_comm, "audio_hal handle is null", -1);
    mutex_lock(hsensor_comm->hsensor_comm_lock);
    ESP_LOGI(TAG, "Codec mode is %d, Ctrl:%d", mode, audio_hal_state);
    ret = hsensor_comm->hsensor_comm_ctrl(mode, audio_hal_state);
    mutex_unlock(hsensor_comm->hsensor_comm_lock);
    return ret;
}

*/
esp_err_t hsensor_comm_send(hsensor_comm_handle_t hsensor_comm, uint8_t *pdata,uint16_t plen)
{
    esp_err_t ret = 0;

    mutex_lock(hsensor_comm->hsensor_comm_lock);
    ret = hsensor_comm->hsensor_comm_send(pdata, plen);
    mutex_unlock(hsensor_comm->hsensor_comm_lock);
    return ret;
}

static uint8_t txFrame[RSP_FRAME_LENGTH];


void generalRspFrame(hsensor_comm_handle_t hcomm,uint8_t code,uint8_t rspd0,uint8_t rspd1)
{
    uint8_t scrc = 0;
    scrc ^= rspd0;
    scrc ^= rspd1;
    txFrame[0] = START_DATA_HEADER;
    txFrame[1] = code;

    txFrame[2] = 2;
    txFrame[3] = rspd0;
    txFrame[4] = rspd1;

    //  txFrame[5] = (uint8_t)(scrc>>8);
    txFrame[5] = scrc;
    txFrame[6] = 0x0a;
    if(hcomm)
        hsensor_comm_send(hcomm, txFrame,7);

}

void generalSendFrame(hsensor_comm_handle_t hcomm,uint8_t code,uint8_t *data,uint16_t dlen)
{
    uint8_t scrc = 0;
    uint16_t i ;
    if((dlen >511)||(hcomm == (hsensor_comm_handle_t)(0)))
        return;

    txFrame[0] = START_DATA_HEADER+(dlen>>8);
    txFrame[1] = code;

    txFrame[2] = (uint8_t)dlen;
    for(i=0; i<dlen; i++)
    {
        txFrame[3+i] = data[i];
        scrc ^= data[i];
    }

    txFrame[4+dlen] = scrc;
    txFrame[5+dlen] = 0x0a;

    hsensor_comm_send(hcomm, txFrame,dlen+5);

}


