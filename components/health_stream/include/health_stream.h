#ifndef _HEALTH_STREAM_H_
#define _HEALTH_STREAM_H_




typedef enum {
	
	IDLE_STATE =0,
	DATA_STREAMING_STATE,	
	SENSOR_FW_UPGRADING_STATE,
	OTA_STATE
}__stream_state;


void stream_process(xQueueHandle sQueue,hsensor_comm_handle_t huart,hsensor_comm_handle_t htcp,hsensor_comm_handle_t hble);


#endif /*_HEALTH_STREAM_H_*/
