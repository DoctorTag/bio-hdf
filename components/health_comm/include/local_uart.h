#ifndef _LOCAL_UART_H_
#define _LOCAL_UART_H_

//#define LUART_BUF_SIZE  (1024)

esp_err_t  local_uart_send(uint8_t *pdata,uint16_t plen);


esp_err_t  local_uart_init(QueueHandle_t xQueue_Req);


esp_err_t  local_uart_deinit();
#endif
