#ifndef ANALYSIS_PIEZO_H__
#define ANALYSIS_PIEZO_H__

#include "stdbool.h"

#define ANALYSIS_TEST_ON  0		 /*Don't change the value */

#define SAMPLE_HZ        80      /*Don't change the value */
#define MAX_HR_MINUTE    300      /*Don't change the value */
#define MAX_RESP_MINUTE  60      /*Don't change the value */



#define ANA_BUF_SECS  (9)	   /*Don't change the value */

#define ANA_BUF_LEN  (SAMPLE_HZ*ANA_BUF_SECS)

typedef enum
{
	OUT_OF_BED = 0,
	BODY_REPOSE,
	BODY_MOVE,
	BODY_UNKNOW
} Body_status;


typedef enum
{
	DEEP_SLEEP = 0,
	LIGHT_SLEEP,
	WAKE_CALM
} Sleep_status;

typedef enum
{
	POSTURE_SUPINE = 0,
	POSTURE_LATERALT

} Sleeping_posture;


typedef struct
{
	Body_status cur_body_status;
	Sleeping_posture sleep_post;
	Sleep_status  cur_sleep_status;
	short hr;
	char  resp;
	bool hr_ok;
	bool resp_ok; 

#if ANALYSIS_TEST_ON	
	///// only test 
	
	uint32_t moved_Intensity; 

	/*  HR */   
	int  hr_raw_dbuf[SAMPLE_HZ];	  /*  HR raw data buf length is SAMPLE_HZ */
	float hr_filted_dbuf[SAMPLE_HZ];   /*  HR filted data buf length is SAMPLE_HZ */
	float hr_enhanced_dbuf[SAMPLE_HZ];  /*  HR enhanced data buf length is SAMPLE_HZ */
	unsigned char hr_peak_locbuf[MAX_HR_MINUTE/60 +1];	/* space length is (MAX_HR_MINUTE/60 +1) */	
	unsigned char hr_ppoints;		 /* maximum is (MAX_HR_MINUTE/60 +1) */ 

		/*  RESP */   
	float resp_dbuf[ANA_BUF_LEN] ;   /*  RESP data buf length is ANA_BUF_LEN */
	unsigned char resp_peak_locbuf[ANA_BUF_SECS +1];	/* space length is (ANA_BUF_SECS +1) */	
	unsigned char resp_ppoints;		 /* maximum is (ANA_BUF_SECS +1) */ 

  	///// 
	
	
#endif
} analysis_result_t;


void analysis_piezo_init(uint32_t raw_peak_threshold) ;
bool analysis_piezo_all(int pie_data,analysis_result_t * a_result) ;

#endif
