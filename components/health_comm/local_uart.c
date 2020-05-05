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
#include "driver/uart.h"

#include "health_board.h"

#include "health_hw.h"
#include "health_comm.h"
#include "local_uart.h"
#include "health_comm_protocol.h"

#ifdef CONFIG_HLOCAL_UART

static const char *TAG = "FeelKit_Uart";

static unsigned char rx_loop = false;

static xQueueHandle fl_uart_queue;

#define FL_UART_NUM CONFIG_HLOCAL_UART
#define FL_RX_BUF_SIZE      (384)

hsensor_comm_protocol_t local_uart_rx_pc;


static void local_uart_rx_task()
{
    uint8_t* data = (uint8_t*) malloc(FL_RX_BUF_SIZE+1);
    uart_event_t event;
    hcomm_RecvInit(&local_uart_rx_pc);
    ESP_LOGI(TAG, "local_uart_rx_task \n");

    while (rx_loop)
    {
        //Waiting for UART event.
        if(xQueueReceive(fl_uart_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {

            switch(event.type)
            {
                case UART_DATA:
                {
                    const int rxBytes = uart_read_bytes(FL_UART_NUM, data, FL_RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
                    if (rxBytes > 0)
                    {
                        ESP_LOGI(TAG, "local_uart_rx dlen=%d,first byte= 0x%x \n",rxBytes,data[0]);
                        for(int i=0; i<rxBytes; i++)
                            hcomm_RecvFrame( &local_uart_rx_pc,data[i]);
                    }
                }
                break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(FL_UART_NUM);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(FL_UART_NUM);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }
    }

    free(data);

    vTaskDelete(NULL);
}


int local_uart_receive(uint8_t *pdata,uint32_t plen,int wait)
{
    return uart_read_bytes(FL_UART_NUM, pdata, plen, wait);
}

esp_err_t  local_uart_send(uint8_t *pdata,uint16_t plen)
{
    uart_write_bytes(FL_UART_NUM, (const char*)pdata, plen);
    return 0;
}

esp_err_t  local_uart_ctrl(uint8_t *pdata,uint32_t plen)
{

    return 0;
}
static QueueHandle_t local_uart_cb_queue;

static esp_err_t  local_uart_framecb(uint8_t *dframe)
{
    hdata_type_t creq;

    creq.type = REQ_FROM_UART;
    creq.pdata = (void*)dframe;

    xQueueSend (local_uart_cb_queue, (void * )&creq, 0);
	return 0;
}
esp_err_t  local_uart_init(QueueHandle_t xQueue_Req)
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config =
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(FL_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(FL_UART_NUM, H_UART_TX_PIN, H_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(FL_UART_NUM, FL_RX_BUF_SIZE * 2, FL_RX_BUF_SIZE * 2, 10, &fl_uart_queue, 0);
    rx_loop = true;
    local_uart_rx_pc.frame_cb = local_uart_framecb;
    local_uart_cb_queue = xQueue_Req;


    xTaskCreate(local_uart_rx_task, "local_uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    ESP_LOGI(TAG, "local_uart_init \n");


    return 0;

}

esp_err_t  local_uart_deinit()
{
    rx_loop = false;
    uart_driver_delete(FL_UART_NUM);
    return 0;

}

#endif
