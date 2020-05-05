
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
#include "health_hw_uart.h"
#include "health_comm_protocol.h"

#ifdef CONFIG_ESP_HEALTH_UART

static const char *TAG = "FeelKit_HUart";

static unsigned char hrx_loop = false;

static xQueueHandle h_uart_queue ;

//static hsensor_comm_protocol_t sensor_uart_rx_pc;
//static health_hw_uart_cb repeat_cb = 0;

#define HEALTH_UART_NUM CONFIG_HCOMM_UART
#define HEALTH_URX_BUF_SIZE      (256)


#define HEALTH_UFRAME_TXBUF_SIZE       (134)  /* head+cmd+131(offset+128data+1crc)+crc*/

#define HEALTH_UFRAME_RXBUF_SIZE      (7)

unsigned char *health_txfrme_buf;
unsigned char health_rxfrme_buf[HEALTH_UFRAME_RXBUF_SIZE];

QueueHandle_t  *hUTqueue = 0;

static void health_hw_uart_rx_task()
{
	uart_event_t event;
	ESP_LOGI(TAG, "health_hw_uart_rx_task \n");

	while (hrx_loop)
	{
		//Waiting for UART event.
		if(xQueueReceive(h_uart_queue, (void * )&event, (portTickType)portMAX_DELAY))
		{

			switch(event.type)
			{
			case UART_DATA:
			{
				hdata_type_t sen_event ;
				if  (hUTqueue)
				{

					sen_event.type = REQ_UART_FOR_SENSOR;

					xQueueSend(*hUTqueue, &sen_event, 0) ;

				}
			}
			break;
			//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGI(TAG, "hw fifo overflow\n");
				//If fifo overflow happened, you should consider adding flow control for your application.
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(HEALTH_UART_NUM);
				break;
			//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG, "ring buffer full\n");
				//If buffer full happened, you should consider encreasing your buffer size
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(HEALTH_UART_NUM);
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

	vTaskDelete(NULL);
}




esp_err_t  health_hw_uart_init(QueueHandle_t *xQueue_Req)
{

	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config =
	{
		.baud_rate = 57600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(HEALTH_UART_NUM, &uart_config);

	//Set UART log level
	esp_log_level_set(TAG, ESP_LOG_INFO);
	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(HEALTH_UART_NUM, HEALTH_UART_TX, HEALTH_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Install UART driver, and get the queue.
	uart_driver_install(HEALTH_UART_NUM, HEALTH_URX_BUF_SIZE * 2, HEALTH_URX_BUF_SIZE * 2, 200, &h_uart_queue, 0);
	hrx_loop = false;
	//  sensor_uart_rx_pc.frame_cb= health_hw_uart_framecb;
	hUTqueue = xQueue_Req;
	//xTaskCreate(health_hw_uart_rx_task, "health_hw_uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
	ESP_LOGI(TAG, "health_hw_uart_init \n");

	health_txfrme_buf = (unsigned char *)malloc(HEALTH_UFRAME_TXBUF_SIZE );



	return 0;

}

esp_err_t  health_hw_uart_deinit()
{

	free(health_txfrme_buf);
	hrx_loop = false;
	uart_driver_delete(HEALTH_UART_NUM);
	return 0;

}

esp_err_t  health_hw_uart_tx(uint8_t rd, uint8_t reg,uint8_t *txdata,uint8_t txlen)
{
	unsigned char i, lcrc = UFRAME_START_HEAD;

	health_txfrme_buf[0] = UFRAME_START_HEAD+rd;
	lcrc +=rd;

	health_txfrme_buf[1] = reg;
	lcrc ^= reg;

	for(i=0; i<txlen; i++)
	{
		health_txfrme_buf[2+i] = txdata[i];
		lcrc ^= txdata[i];
	}
	health_txfrme_buf[2+txlen]  = lcrc;
	uart_write_bytes(HEALTH_UART_NUM,(const char* )health_txfrme_buf,txlen+3);
	return 0;

}

uint8_t health_hw_uart_crc(void)
{
	uint8_t i, crc = health_rxfrme_buf[0] ;

	for(i=1; i<6; i++)
	{
		crc ^= health_rxfrme_buf[i];
	}
	return (crc == health_rxfrme_buf[6]  );
}

esp_err_t  health_hw_uart_wait_rsp(uint8_t *rxdata,uint8_t *rxlen, portTickType delay,uint8_t rsq_cxt)
{
	uart_event_t event;
	uint8_t len;
	(*rxlen) = 0;
	if(xQueueReceive(h_uart_queue, (void * )&event, delay))
	{
		if(event.type == UART_DATA)
		{
			len = uart_read_bytes(HEALTH_UART_NUM, health_rxfrme_buf, HEALTH_UFRAME_RXBUF_SIZE, 1000 / portTICK_RATE_MS);
			if((len ==7)&&(health_rxfrme_buf[1] == rsq_cxt) && health_hw_uart_crc())
			{

				(*rxdata) = health_rxfrme_buf[2];
				(*rxlen) = 1;

			}
			else
			{
				ESP_LOGI(TAG, "health_hw_uart_wait_rsp error !!! \n");

			}
		}
	}

	return 0;

}

uint8_t*  health_hw_uart_rx(portTickType delay)
{
	uint8_t i = uart_read_bytes(HEALTH_UART_NUM, health_rxfrme_buf, HEALTH_UFRAME_RXBUF_SIZE, 1000 / portTICK_RATE_MS);
	if((i ==7)&&health_hw_uart_crc())
	{
		return health_rxfrme_buf;
	}
	else
	{
		ESP_LOGI(TAG, "health_hw_uart_rx error !!! \n");

	}
	return 0 ;

}

esp_err_t  health_hw_uart_stream(uint8_t start)
{
	if(start&M_FUN_START)
	{
		if(hrx_loop == false)
		{
			hrx_loop = true;
			xTaskCreate(health_hw_uart_rx_task, "health_hw_uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
		}
	}
	else
	{
		if(hrx_loop)
		{
			uart_event_t event;
			event.type = UART_EVENT_MAX;
			hrx_loop = false;
			xQueueSend(h_uart_queue, &event, 0) ;
		}
	}
	return 0;

}


void health_hw_uart_io_reconfig(void)
{


	uart_config_t uart_config =
	{
		.baud_rate = 57600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_driver_delete(HEALTH_UART_NUM);
	uart_param_config(HEALTH_UART_NUM, &uart_config);

	//Set UART log level
	esp_log_level_set(TAG, ESP_LOG_INFO);
	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(HEALTH_UART_NUM, HEALTH_UART_TX, HEALTH_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Install UART driver, and get the queue.
	uart_driver_install(HEALTH_UART_NUM, HEALTH_URX_BUF_SIZE * 2, HEALTH_URX_BUF_SIZE * 2, 200, &h_uart_queue, 0);
	hrx_loop = false;
}
#endif



