/* Play mp3 file by audio pipeline

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "health_board.h"
#include "health_hw.h"
#include "health_comm.h"

#include "health_stream.h"

static const char *FEELKIT_TAG = "FeelKit";


void app_main(void)
{

    QueueHandle_t xQueue_Stream;

    esp_err_t ret;

    sensor_data_t rdata;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    health_hw_init(&xQueue_Stream);

    // esp_log_level_set(FEELKIT_TAG, ESP_LOG_INFO);

    //  health_read_reg(RegNum_Dev_Id, &(rdata.dtype));
    // ESP_LOGI(FEELKIT_TAG, "health sensor id = 0x%x \n",rdata.dtype);


    stream_process(xQueue_Stream,hsensor_comm_init(xQueue_Stream, REQ_FROM_UART),hsensor_comm_init(xQueue_Stream, REQ_FROM_TCP),hsensor_comm_init(xQueue_Stream, REQ_FROM_BLE));
    ESP_LOGI(FEELKIT_TAG, "health run error \n");
    vQueueDelete(xQueue_Stream);

}
