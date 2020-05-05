#ifndef _HEALTH_OTA_H_
#define _HEALTH_OTA_H_

#define HOTA_BUFFSIZE 1024
#define HOTA_FRAME_LEN 128

const esp_partition_t * health_ota_init(esp_ota_handle_t *update_handle );


esp_err_t  health_ota_write(esp_ota_handle_t update_handle ,uint8_t  *rdata,uint16_t rlen);


esp_err_t  health_ota_complete(esp_ota_handle_t update_handle,esp_partition_t *update_partition);

#endif

