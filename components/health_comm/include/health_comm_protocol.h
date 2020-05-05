#ifndef _HEALTH_COMM_PROTOCOL_H_
#define _HEALTH_COMM_PROTOCOL_H_

#define START_DATA_HEADER			0x55


#define  HCP_RX_HEADER           0x00
#define  HCP_RX_CMD         0x01
#define  HCP_RX_LENGTH        0x02
#define  HCP_RX_DATA           0x03
#define  HCP_RX_CRC           0x04
#define  HCP_RX_END            0x05

#define  FRAME_TYPE_MASK            0x70
//#define  CMD_HIGH            0xa0

#define MAX_RDATA_LENGTH 131         // 1SUB_CMD+2offset+128DATA

/*1 Start+1CMD+1DLEN+1subCMD+2offset+128DATA+1CRC+1END*/
#define MAX_RX_LENGTH 136

#define RSP_FRAME_LENGTH 518

typedef struct {
	
unsigned char rx_state;
unsigned short offset;
unsigned char lcrc;
unsigned char rcrc;
unsigned char rlength;
unsigned char rx_frame[MAX_RX_LENGTH];
 esp_err_t (*frame_cb)(unsigned char * dframe);
//__health_req_type  htype;
//QueueHandle_t Queue_Req;

} hsensor_comm_protocol_t;

void hcomm_RecvInit(hsensor_comm_protocol_t *pc);


void  hcomm_RecvFrame( hsensor_comm_protocol_t *pc,unsigned char src);

#endif
