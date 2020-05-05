#ifndef _HEALTH_TCP_SERVER_H_
#define _HEALTH_TCP_SERVER_H_


/* 
*/
uint8_t*  health_tcp_getIp(void);

void  health_tcp_connect(char * host_ip);
esp_err_t  health_tcp_send(uint8_t *pdata,uint16_t plen);


esp_err_t  health_tcp_init(QueueHandle_t xQueue_Req);


esp_err_t  health_tcp_deinit();

/* 
*/

#endif

