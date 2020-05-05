/*
*/
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "health_hw.h"
#include "health_comm.h"
#include "health_comm_protocol.h"

static const char *TAG = "FeelKit_Protocol";

/*****************************************************************************************

*****************************************************************************************/


void hcomm_RecvInit(hsensor_comm_protocol_t *pc)
{
    if(pc)
    {
        pc->rx_state = HCP_RX_HEADER;
        pc->offset = 0;
        pc->rlength = 0;
    }
}


void  hcomm_RecvFrame( hsensor_comm_protocol_t *pc,unsigned char src)
{
    if(pc)
    {
        pc->rx_frame[ pc->offset] = src ;
        pc->offset ++;
        switch(pc->rx_state)
        {
            case HCP_RX_HEADER:
                if(pc->rx_frame[pc->offset-1] == START_DATA_HEADER)
                {
                    pc->rx_state = HCP_RX_CMD;
                    pc->offset = 1;

                }
                else
                {
                    hcomm_RecvInit(pc);
                }

                break;
            case HCP_RX_CMD:
                // if((CMD_MASK & pc->rx_frame[pc->offset-1]) == CMD_HIGH)
                //  {
                pc->rx_state = HCP_RX_LENGTH;

                //  }
                // else
                // {
                //     hcomm_RecvInit(pc);
                //  }
                break;
            case HCP_RX_LENGTH:
                pc->rlength = pc->rx_frame[pc->offset-1];
                if(pc->rlength <= MAX_RDATA_LENGTH)
                {
                    pc->rx_state = HCP_RX_DATA;
                    pc->lcrc = 0;
                }
                else
                {
                    hcomm_RecvInit(pc);
                }
                break;
            case HCP_RX_DATA:
                if(pc->offset >= (pc->rlength+3))
                {
                    pc->rx_state = HCP_RX_CRC;

                }
                pc->lcrc ^= pc->rx_frame[pc->offset-1];
                break;
            case HCP_RX_CRC:
                if(pc->offset == (pc->rlength+4))
                {
                    pc->rcrc = pc->rx_frame[pc->offset-1];
                    pc->rx_state = HCP_RX_END;
                }


                if(pc->offset > (pc->rlength+4) )
                {
                    hcomm_RecvInit(pc);
                }
                break;

            case HCP_RX_END:
                if(pc->rx_frame[pc->offset-1] == 0x0a)
                {
                    //ESP_LOGI(TAG, "rx frame ok \n");
                    /*
                                        if  (pc->Queue_Req)
                                        {
                                            health_req_t creq;
                                            creq.type = pc->htype;
                                            creq.pdata = pc->rx_frame;
                                            pc->rx_frame[0] = (pc->lcrc);
                                            xQueueSend (pc->Queue_Req, (void * )&creq, 0);
                                        }
                                        */
                    if  (pc->frame_cb)
                    {

                        pc->rx_frame[0] = (pc->lcrc);
                        pc->frame_cb(pc->rx_frame);
                        //xQueueSend (pc->Queue_Req, (void * )&creq, 0);
                    }

                    hcomm_RecvInit(pc);
                }
                else
                {
                    if(pc->offset >  (pc->rlength+5))
                    {
                        hcomm_RecvInit(pc);
                    }

                }
                break;

        }
    }
}




