/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _HEALTH_HW_UART_H_
#define _HEALTH_HW_UART_H_

#ifdef __cplusplus
extern "C" {
#endif
#define UFRAME_START_HEAD  0x58
#define UFRAME_START_RD  0x01
#define UFRAME_START_WR  0x00


esp_err_t  health_hw_uart_tx(uint8_t rd, uint8_t reg,uint8_t *txdata,uint8_t txlen);
esp_err_t  health_hw_uart_wait_rsp(uint8_t *rxdata,uint8_t *rxlen, portTickType delay,uint8_t rsq_cxt);
uint8_t*  health_hw_uart_rx(portTickType delay);
esp_err_t  health_hw_uart_stream(uint8_t start);
esp_err_t  health_hw_uart_init(QueueHandle_t *xQueue_Req);
void health_hw_uart_io_reconfig(void);


esp_err_t  health_hw_uart_deinit();
uint8_t health_hw_uart_crc(void);


#ifdef __cplusplus
}
#endif

#endif
