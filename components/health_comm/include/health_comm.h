#ifndef _HEALTH_COMM_H_
#define _HEALTH_COMM_H_

#define  MASK_8BIT     2
#define  MASK_12BIT   3
#define  MASK_16BIT   4
#define  MASK_24BIT   6

#define WRITE_REG_COMMAND			0x11
#define READ_REG_COMMAND			0x12
#define DATA_STREAMING_COMMAND	0x13
#define DATA_STREAMING_PACKET		0x13
#define ACQUIRE_DATA_COMMAND		0x14
#define ACQUIRE_DATA_PACKET 		0x14
#define DEVICE_ID_REQ                   	0x15
#define DATA_DOWNLOAD_COMMAND	0x16
//#define FIRMWARE_UPGRADE_CMD	       0x17
#define FIRMWARE_UPGRADING_DATA	0x18
#define FIRMWARE_VERSION_REQ		0x19
#define STATUS_INFO_REQ 			0x1A
#define FILTER_SELECT_COMMAND		0x1B
#define ON_HSENSOR_COMMAND		0x1C
#define RESTART_COMMAND				0x1D
#define START_RECORDING_COMMAND		0x1E
#define SENSOR_BUSY_IND		       0x1F

#define DATA_TYPE_DEFAULT                0x20
#define DATA_TYPE_8BIT	                     DATA_TYPE_DEFAULT	
#define DATA_TYPE_12BIT	              (DATA_TYPE_DEFAULT+((MASK_12BIT-MASK_8BIT)<<4))
#define DATA_TYPE_16BIT	               (DATA_TYPE_DEFAULT+((MASK_16BIT-MASK_8BIT)<<4))
#define DATA_TYPE_24BIT	               (DATA_TYPE_DEFAULT+((MASK_24BIT-MASK_8BIT)<<4))

#define DATA_TYPE_MASK		              0x0F


#define SET_WIFI_CONFIG			0x30
#define IPADDR_IND			0x31
#define INBED_IND                 0x32


#define GET_HR			0x50
#define GET_TEMP			0x51
#define GET_IMP			0x52
#define GET_SPO2			0x53
#define GET_RESP			0x54
#define GET_ACTIVITY			0x55




#define NORMAL_RST				       0x01
#define FWUPGRADE_RST				0x02
#define FWUPGRADE_PWD		0x58

/* cmd respone info */

#define RSP_OK				        0x01
#define RSP_ERROR_UNKNOW	        0x02
#define RSP_ERROR_CRC		        0x03
#define RSP_ERROR_STATUS		 0x04
#define RSP_ERROR_NOMEM		 0x05


#define H_INFO_IPADDR      0x20
#define H_INFO_INBED      0x21


typedef struct hsensor_comm* hsensor_comm_handle_t;


hsensor_comm_handle_t hsensor_comm_init(QueueHandle_t xQueue, int index);


esp_err_t hsensor_comm_deinit(hsensor_comm_handle_t hsensor_comm, int index);


/*
esp_err_t hsensor_comm_ctrl(hsensor_comm_handle_t hsensor_comm, audio_hal_codec_mode_t mode, audio_hal_ctrl_t audio_hal_state)


*/
esp_err_t hsensor_comm_send(hsensor_comm_handle_t hsensor_comm, uint8_t *pdata,uint16_t plen);

void generalRspFrame(hsensor_comm_handle_t hcomm,uint8_t code,uint8_t rspd0,uint8_t rspd1);
void generalSendFrame(hsensor_comm_handle_t hcomm,uint8_t code,uint8_t *data,uint16_t dlen);

#endif
