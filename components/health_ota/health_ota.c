#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "health_ota.h"

static const char *TAG = "FeelKit_OTA";


const esp_partition_t * health_ota_init(esp_ota_handle_t *update_handle )
{
    esp_err_t err;
    const esp_partition_t *update_partition ;

    ESP_LOGI(TAG, "Starting OTA example...");

   const esp_partition_t *configured = esp_ota_get_boot_partition();
  const   esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);


    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    if(update_partition == NULL)
        return NULL;

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        return NULL;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");


    return update_partition ;
}

esp_err_t  health_ota_write(esp_ota_handle_t update_handle ,uint8_t  *rdata,uint16_t rlen)
{
 esp_err_t err = ESP_ERR_INVALID_ARG;
   if(rdata)
   	{
        err = esp_ota_write( update_handle, (const void *)rdata, rlen);
       
        
        ESP_LOGD(TAG, "Written image length %d", rlen);
   	}
 return err;  
      
     
}

esp_err_t  health_ota_complete(esp_ota_handle_t update_handle,esp_partition_t *update_partition)
{
    esp_err_t err = esp_ota_end(update_handle) ;
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        return err;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ESP_OK;
}
